#!/usr/bin/env python

'''
Performs an optimization to determine the best points for a 3 point
turn-around maneuver. The starting point for the maneuver is the ending point
for the obstacle avoidance path and the final point of the maneuver is the
starting point for the grasping path generation.
'''


import rospy
from geometry_msgs.msg import PoseStamped, Pose
from grasping.srv import OptimizeManeuver, OptimizeManeuverRequest, OptimizeManeuverResponse
from motion_testing.msg import PathWithGear
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
from scipy.spatial import ConvexHull
from scipy.optimize import minimize
import math
from inPolygon import pointInPolygon


class Pose2D:
    def __init__(self, x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta

    def __str__(self):
        rep = "Pose:\n   x: {0:0.04f}\n   y: {1:0.04f}\n   theta: {2:0.04f}".format(self.x, self.y, self.theta)
        return rep

def wrapToPi(angle):
    '''
    Wraps an angle in radians between [-pi, pi].

    Args:
    -----
    angle: angle in radians to be wrapped

    Returns:
    --------
    the angle now wrapped within the range [-pi, pi]
    '''
    return (angle + np.pi) % (2*np.pi) - np.pi

def rotZ2D(theta):
    '''
    Returns a 2D rotation matrix going from frame 2 to frame 1 where frame 2 is
    rotated about the frame 1 origin by theta radians
          x_2
         /
        /
       /
      /
     /\
    /  | theta
    ----------------- x_1

    x_1 = R*x_2

    Args:
    -----
    theta: angle frame 2 has been rotated about frame 1

    Returns:
    --------
    R: 2x2 rotation matrix as numpy array
    '''
    R = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])

    return R

class ManeuverPath:
    def __init__(self):
        #=============#
        # ROS Objects
        #=============#
        # ROS Parameters
        rospy.init_node("maneuver_path")
        self.approach_pose_offset = rospy.get_param("~approach_pose_offset", 6.0) # distance in meters used to offset the maneuver starting pose from the roll, this is to give a good initial value for the optimization
        self.roll_radius = rospy.get_param("/roll/radius", 0.20)


        self.target_x = None
        self.target_y = None
        self.target_approach_angle = None
        self.obstacles = None
        self.maneuver_path = PathWithGear()
        self.maneuver_path.path.header.frame_id = "/odom"
        self.optimization_success = False
        self.current_pose = Pose()
        self.rate = rospy.Rate(30)

        # Forklift dimensions
        self.base_to_clamp = rospy.get_param("/forklift/body/base_to_clamp", 1.4658) # {m}
        self.base_to_back = rospy.get_param("/forklift/body/base_to_back", 1.9472) # {m}
        self.width = rospy.get_param("/forklift/body/width", 1.3599) # {m}
        self.total_length = rospy.get_param("/forklift/body/total", 3.5659) # {m}
        self.buffer = 1.0 # {m} gap between edge of forklift and the bounding box

        '''
        Forklift Bounding Box
                                   L_3
                             |--------------|
                            ___             ___
                             | buffer        |
                            _|_              |
                         /                   |
                         \       /           |
                           \   /             | L_1
                     -----------------       |
                     |__     ^     __|       |
                     |  |    |    |  |       |
                     |__| <--o    |__|buffer---
                     |               |-----| |
                     |               |       |
                     |               |       |
                     |               |       | L_2
                     -----------------       |
                             | buffer        |
                            _|_             _|_


        '''
        self.L_1 = self.base_to_clamp + self.buffer # length from base to back end of box
        self.L_2 = self.base_to_back + self.buffer # length from base to front end of box
        self.L_3 = (self.width/2.) + self.buffer # length from base to side

        # Max turning radius
        self.axle_distance = 1.7249
        self.max_angle = 70 # deg
        self.min_radius = self.axle_distance/np.tan(self.max_angle*(np.pi/180.0))

        self.resolution = 0.10 # map resolution, updates with each OccupancyGrid map callback

        # ROS Publishers and Subscribers
        self.occupancy_grid_sub = rospy.Subscriber("/map", OccupancyGrid, self.occupancyGridCallback, queue_size=1)
        self.roll_pose_sub = rospy.Subscriber("/roll/pose", PoseStamped, self.rollCallback, queue_size=3)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odomCallback, queue_size=1)
        self.path1_pub = rospy.Publisher("~path1", PathWithGear, queue_size=3)
        self.path2_pub = rospy.Publisher("~path2", PathWithGear, queue_size=3)
        self.approach_pose_pub = rospy.Publisher("/forklift/approach_pose", PoseStamped, queue_size=3)
        # indicates whether the optimzation completed successfully or not, to know whether the path is usable
        self.optimize_maneuver_srv = rospy.Service("~optimize_maneuver", OptimizeManeuver, self.optimizeManeuver)

    def spin(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

    def maneuverPoses(self, pose_s, r_1, alpha_1, r_2, alpha_2):
        '''
        Calculates the middle and final poses of the turn-around maneuver given
        the starting position and the desired turning radius and arc length of
        the two segments.

        Args:
        -----
        pose_s: Pose2D object containing (x,y) position and yaw angle, theta
        r_1: turning radius of the first section (positive means steering wheel
             is turned to the left, negative is turned to the right)
        alpha_1: arc angle of the first section (positive means traveling in the
                 forkward direction, negative is backwards))
        r_2: turning radius of the second section (should be positive, will be
             made the opposite sign of r_1)
        alpha_2: arc angle of the second section (should be positive, will be
                 made the oppposite sign of alpha_1)

        Returns:
        --------
        [pose_m, pose_f]: the middle and final pose values as Pose2D objects
        '''
        # Unpack pose values
        x_s = pose_s.x
        y_s = pose_s.y
        theta_s = pose_s.theta

        # Calculate parameters for the middle pose
        theta_m = theta_s + np.sign(r_1)*alpha_1
        pos_s = np.array([x_s, y_s])
        R1 = rotZ2D(theta_s)
        pos_m = pos_s + R1.dot(np.array([np.abs(r_1)*np.sin(alpha_1), r_1*(1 - np.cos(alpha_1))]))

        # Make second parameters opposite signs of the first
        r_2 = -np.sign(r_1)*r_2
        alpha_2 = -np.sign(alpha_1)*alpha_2

        # Calculate parameters for the final pose
        theta_f = theta_m + np.sign(r_2)*alpha_2
        R2 = rotZ2D(theta_m)
        pos_f = pos_m + R2.dot(np.array([np.abs(r_2)*np.sin(alpha_2), r_2*(1 - np.cos(alpha_2))]))

        # Save middle and final pose as Pose2D objects
        pose_m = Pose2D(pos_m[0], pos_m[1], theta_m)
        pose_f = Pose2D(pos_f[0], pos_f[1], theta_f)

        return [pose_m, pose_f]

    def maneuverLength(self, r_1, alpha_1, r_2, alpha_2):
        '''
        Calculates the length of the turn-around maneuver.

        Args:
        -----
        r_1: turning radius of the first section (can be positive or negative)
        alpha_1: arc length of the first section (positive only)
        r_2: turning radius of the second section (can be positive or negative)
        alpha_2: arc length of the second section (positive only)

        Returns:
        --------
        L: the length of the path
        '''
        L = np.abs(r_1)*np.abs(alpha_1) + np.abs(r_2)*np.abs(alpha_2)
        return L

    def maneuverSegmentPath(self, pose, r, alpha):
        '''
        Takes the length of the path and determines the number of points to add
        between the endpoints of the arc length defined by the radius, 'r', and
        arc angle, 'alpha'. Then it creates a list of points to be used as a
        path. Note: the starting point is not included in the path, this should
        be the endpoint of the previous path. This way the paths can be
        concatenated together without overlapping points.

        Args:
        -----
        pose: starting pose of arc as Pose2D object
        r: arc radius {m}
        alpha: arc angle {rad}

        Returns:
        --------
        path: list containing [x,y] pairs of points along the arc
        '''
        arc_length = abs(r*alpha)
        num_points = int(math.ceil(arc_length/self.resolution))
        if (num_points == 0):
            num_points = 1
        delta_alpha = float(alpha)/(num_points)

        start_position = np.array([pose.x, pose.y])
        R = rotZ2D(pose.theta)
        path = []
        for i in range(1, num_points+1):
            path_point = start_position + R.dot(np.array([np.abs(r)*np.sin(i*delta_alpha), r*(1 - np.cos(i*delta_alpha))]))
            path.append([path_point[0], path_point[1]])

        return path


    def forkliftBoundingBox(self, pose):
        '''
        Calculates the four corners of the forklift's bounding box and
        returns them as a 4x2 numpy array

        Args:
        -----
        pose: Pose2D object containing the forklifts position (x,y) and yaw
              angle, theta

        Returns:
        --------
        points: a 4x2 numpy array containing the bounding box points, first
                point is in the back right corner and goes counter-clockwise
        '''
        # Unpack the pose
        x = pose.x
        y = pose.y
        theta = pose.theta

        # Corner points in forklift frame
        corners = [np.array([-self.L_2,-self.L_3]),
                   np.array([ self.L_1,-self.L_3]),
                   np.array([ self.L_1, self.L_3]),
                   np.array([-self.L_2, self.L_3])]

        R_forklift = rotZ2D(theta)

        points = []
        for i in range(len(corners)):
            points.append(np.array([x, y]) + R_forklift.dot(corners[i]))

        return np.asarray(points)

    def maneuverBoundingBox(self, pose_s, pose_m, pose_f):
        # TODO: This code needs to be updated in the future to handle
        # situtations where the turn-around procedure may not be internal to the
        # endpoints. If the arc angle is large, then there will be points that
        # are not considered along the path with regards to obstacle collision.
        # Currently it just takes the three endpoints of the maneuver (start,
        # middle, final) and generates the convex hull of the set of forklift
        # bounding box points for each of those poses.
        '''
        Generates the convex hull for the set of bounding box points of the
        three maneuver end points (start, middle, final).

        Args:
        -----
        pose_s: starting pose as Pose2D object
        pose_m: middle pose as Pose2D object
        pose_f: final pose as Pose2D object

        Returns:
        --------
        points: mx2 numpy array containing all the points of the bounding box,
                size may vary based on poses, anticipate at least 6 points,
                Note: the starting point is added to the end of the array to
                make a "closed" polygon
        '''
        # Generate full list of bounding box points
        points_s = self.forkliftBoundingBox(pose_s)
        points_m = self.forkliftBoundingBox(pose_m)
        points_f = self.forkliftBoundingBox(pose_f)

        all_points = np.vstack((points_s, points_m, points_f))

        # Find the convex hull points
        hull = ConvexHull(all_points)
        points = hull.points[hull.vertices]

        # Add the starting point to the end of the array to make it a "closed"
        # polygon
        points = np.vstack((points, points[0,:]))

        return points

    def rowMajorIndex(self, i, width):
        '''
        Takes a single index 'i' of a 1D array representing a Row-Major matrix
        and determines the 2D indices. The top left cell is considered (0,0).

        Args:
        -----
        i: 1D index
        width: number of cells in a single row of the matrix

        Returns:
        --------
        [row,col]: 2D indices of the row-major matrix
        '''
        row = math.floor(i/width)
        col = i % width

        return [row, col]

    def gridMapToObstacles(self, grid):
        '''
        Converts an OccupancyGrid message into a numpy array of obstacle
        positions.

        Args:
        -----
        grid: an OccupancyGrid ROS message

        Returns:
        --------
        obstacles: mx2 numpy array of obstacle positions
        '''
        # Get map parameters
        width = grid.info.width
        resolution = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y

        # Iterate through each cell in the grid and determine the obstacle probability
        obstacles = []
        for i in range(len(grid.data)):
            # If there is an obstacle, convert index into row & column
            if (grid.data[i] > 0):
                [row, col] = self.rowMajorIndex(i, width)

                # Convert row and column into a position in map frame
                # The map is in "image frame", meaning that the rows are along the Y
                # axis and the columns are along the X axis with the Z axis going
                # "into" the matrix.
                x = (col*resolution + resolution/2.0) + origin_x
                y = (row*resolution + resolution/2.0) + origin_y
                obstacles.append([x,y])

        obstacles = np.asarray(obstacles)

        return obstacles

    def obstaclesInManeuver(self, pose_s, pose_m, pose_f, obstacles):
        '''
        Outputs the number of obstacles present in the maneuver bounding box.

        Args:
        -----
        pose_s: starting pose as Pose2D object
        pose_m: middle pose as Pose2D object
        pose_f: final pose as Pose2D object
        obstacles: mx2 numpy array of 2D points representing the position of
                   obstacles

        Returns:
        --------
        num_obstacles: number of obstacles within the maneuver bounding box
        '''
        # Create polygon of points representing the bounding box
        polygon = self.maneuverBoundingBox(pose_s, pose_m, pose_f)

        # Determine which points are inside the boundary
        [points_in, points_on] = pointInPolygon(obstacles, polygon)
        num_obstacles = np.sum(points_in)

        return num_obstacles

    def maneuverObjective(self, x, params):
        '''
        Optimization objective function that calculates the cost associated with
        generating the maneuver path for approaching the roll. Takes the design
        variables as well as the parameters. An additional wrapper function
        should be created that preassigns the parameters before passing the
        function to the optimizer.

        Args:
        -----
        x: [x_s, y_s, theta_s, r_1, alpha_1, r_2, alpha_2]
        x_s: forklift starting 'x' position of maneuver
        y_s: forklist starting 'y' position of maneuver
        theta_s: forklift starting 'yaw' angle of maneuver
        r_1: turning radius of initial maneuver segment (can be positive or
             negative, must be bounded by the radius of the max steering angle)
        alpha_1: arc angle of initial maneuver segment (can be positive or
                 negative)
        r_2: turning radius of second maneuver segment (must be positive so its
             sign can be adjusted to be opposite of r_1, only the magnitude of
             r_2 is important)
        alpha_2: arc angle of second maneuver segment (must be positive so its
                 sign can be adjusted to be opposite of alpha_1, only the
                 magnitude of alpha_2 is important)

        params: {"current_pose" : pose, "forklift_length" : L_f, "weights" :
                 [w_1, w_2, w_3]}
        "current_pose" : [pose]: the forklift's current pose provided as Pose2D
                         object containing the (x,y) position and yaw angle, theta.
        "forklift_length" : L_f: length of the forklift from clamp tip to back.
        "weights" : [w_1, w_2, w_3, w_4]: weights for the cost, w_1 = weight for
                    distance error between the approach point and the point one
                    forklift length forward from the final pose of the
                    maneuver, w_2 = weight for the maneuver length, w_3 =
                    weight for the distance error between the maneuver starting
                    position and the forklift's current position, w_4 = weight for the angle error between forklift approach orientation and the roll approach orientation (ideally should be PI radians off from each other)

        Returns:
        --------
        J: the cost of the maneuver using the given design variables
        '''
        # Unpack variables and parameters
        x_s = x[0]
        y_s = x[1]
        theta_s = x[2]
        r_1 = x[3]
        alpha_1 = x[4]
        r_2 = x[5]
        alpha_2 = x[6]
        current_pose = params["current_pose"]
        L_f = params["forklift_length"]
        weights = params["weights"]

        # Covnert state to pose
        pose_s = Pose2D(x_s, y_s, theta_s)

        # Get the maneuver poses based on the current design variables
        [pose_m, pose_f] = self.maneuverPoses(pose_s, r_1, alpha_1, r_2, alpha_2)

        # Unpack poses
        x_f = pose_f.x
        y_f = pose_f.y
        theta_f = wrapToPi(pose_f.theta)

        # Calculate the maneuver length
        maneuver_length = self.maneuverLength(r_1, alpha_1, r_2, alpha_2)

        # Calculate the second point of the B-spline approach curve and get the distance from the third point
        point3 = [self.target_x + (self.roll_radius + self.base_to_clamp + self.total_length)*np.cos(self.target_approach_angle), self.target_y + (self.roll_radius + self.base_to_clamp + self.total_length)*np.sin(self.target_approach_angle)]
        point2 = [L_f*np.cos(theta_f) + x_f, L_f*np.sin(theta_f) + y_f]
        approach_error = np.sqrt((point3[0] - point2[0])**2 + (point3[1] - point2[1])**2)

        # Calculate the distance between the maneuver start pose and the current forklift pose
        start_error = np.sqrt((current_pose[0] - x_s)**2 + (current_pose[1] - y_s)**2)

        # Approach angle error
        angle_error = (wrapToPi(theta_f - (self.target_approach_angle + np.pi)))**2

        # Calculate the weighted cost
        J = weights[0]*approach_error + weights[1]*maneuver_length + weights[2]*start_error + weights[3]*angle_error

        return J

    def maneuverIneqConstraints(self, x, params):
        '''
        Optimization inequality constraint function. scipy.optimize.minimize
        defines the constraints as c_j >= 0.

        subject to:
        1) -obstaclesInManeuver() >= 0
        2) abs(r_1) - min_radius >= 0

        Args:
        -----
        x: [x_s, y_s, theta_s, r_1, alpha_1, r_2, alpha_2]
        x_s: forklift starting 'x' position of maneuver
        y_s: forklist starting 'y' position of maneuver
        theta_s: forklift starting 'yaw' angle of maneuver
        r_1: turning radius of initial maneuver segment (can be positive or
             negative, must be bounded by the radius of the max steering angle)
        alpha_1: arc angle of initial maneuver segment (can be positive or
                 negative)
        r_2: turning radius of second maneuver segment (must be positive so its
             sign can be adjusted to be opposite of r_1, only the magnitude of
             r_2 is important)
        alpha_2: arc angle of second maneuver segment (must be positive so its
                 sign can be adjusted to be opposite of alpha_1, only the
                 magnitude of alpha_2 is important)

        params: {"obstacles" : obstacles, "min_radius" : min_radius}
        "obstacles" : obstacles: mx2 numpy array of obstacle locations (x,y)
        "min_radius" : min_radius: the minimum allowable turning radius

        Returns:
        --------
        C: (2,) numpy array containing the constraint values
        '''
        # Unpack variables and parameters
        x_s = x[0]
        y_s = x[1]
        theta_s = x[2]
        r_1 = x[3]
        alpha_1 = x[4]
        r_2 = x[5]
        alpha_2 = x[6]

        # Initialize constraints
        C = np.zeros(2)

        # Convert states to poses
        pose_s = Pose2D(x_s, y_s, theta_s)

        # Get the maneuver poses based on the current design variables
        [pose_m, pose_f] = self.maneuverPoses(pose_s, r_1, alpha_1, r_2, alpha_2)

        # Frist constraint
        C[0] = -self.obstaclesInManeuver(pose_s, pose_m, pose_f, params["obstacles"])

        # Second constraint
        C[1] = abs(x[3]) - params["min_radius"]

        return C

    def optimizeManeuver(self, req):
        '''
        Sets up the optimization problem then calculates the optimal maneuver
        poses. Runs when any message is sent to the corresponding subscriber,
        the 'msg' value does not matter.

        Args:
        -----
        msg: ROS Bool message

        Returns:
        --------
        path: the optimal path as a ROS nav_msgs/Path message
        '''
        # Make sure a target pose exists
        if (self.target_x is not None):
            # DEBUG:
            print("Running maneuver optimization...")

            # Set initial guess
            x_s = self.target_x + np.cos(self.target_approach_angle)*self.approach_pose_offset
            y_s = self.target_y + np.sin(self.target_approach_angle)*self.approach_pose_offset
            theta_s = self.target_approach_angle # the starting pose is facing backwards, so the approach angle is the same as the starting orientation rather than adding 180deg
            r_1 = 2
            alpha_1 = -np.pi/2
            r_2 = 2
            alpha_2 = np.pi/2

            # # DEBUG: Print starting path for optimizer
            # pose_s = Pose2D(x_s, y_s, theta_s)
            # [pose_m, pose_f] = self.maneuverPoses(pose_s, r_1, alpha_1, abs(r_2), abs(alpha_2)) # this function expects r_2 and alpha_2 to be positve values
            #
            # path = self.maneuverSegmentPath(pose_s, r_1, alpha_1)
            # path.extend(self.maneuverSegmentPath(pose_m, r_2*-np.sign(r_1), alpha_2*-np.sign(alpha_1)))
            # self.maneuver_path.header.stamp = rospy.Time.now()
            # self.maneuver_path.poses = []
            # for i in range(len(path)):
            #     point = PoseStamped()
            #     point.pose.position.x = path[i][0]
            #     point.pose.position.y = path[i][1]
            #     self.maneuver_path.poses.append(point)
            #
            # self.path_pub.publish(self.maneuver_path)
            # raw_input("Pause")

            x0 = [x_s, y_s, theta_s, r_1, alpha_1, r_2, alpha_2]
            # Set params
            # TODO:
            # add the forklifts current pose from "/odom"
            current_pose2D = Pose2D()
            current_pose2D.x = self.current_pose.position.x
            current_pose2D.y = self.current_pose.position.y
            euler_angles = euler_from_quaternion([self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w])
            current_pose2D.theta = euler_angles[2]

            params = {"current_pose" : [current_pose2D.x,current_pose2D.y,current_pose2D.theta], "forklift_length" : (self.base_to_back + self.base_to_clamp), "weights" : [10, 1, 0.1, 1], "obstacles" : self.obstacles, "min_radius" : self.min_radius}

            # Set up optimization problem
            obj = lambda x: self.maneuverObjective(x, params)
            ineq_con = {'type': 'ineq',
                        'fun' : lambda x: self.maneuverIneqConstraints(x, params),
                        'jac' : None}
            bounds = [(-20, 20),
                      (-20, 20),
                      (-np.pi, np.pi),
                      (-10*np.pi, 10*np.pi),
                      (-np.pi, np.pi),
                      (self.min_radius, 10*np.pi),
                      (np.pi/4, np.pi)]

            # Optimize
            res = minimize(obj, x0, method='SLSQP', bounds=bounds, constraints=ineq_con)

            # DEBUG:
            print("===== Optimization Results =====")
            print("Success: %s" % res.success)
            print("Message: %s" % res.message)
            print("Results:\n  x: %f,  y: %f,  theta: %f\n  r_1: %f,  alpha_1: %f\n  r_2: %f,  alpha_2: %f" % (res.x[0], res.x[1], res.x[2], res.x[3], res.x[4], res.x[5], res.x[6]))

            # Store result
            x_s = res.x[0]
            y_s = res.x[1]
            theta_s = res.x[2]
            r_1 = res.x[3]
            alpha_1 = res.x[4]
            r_2 = res.x[5]
            alpha_2 = res.x[6]

            # NOTE: remember to make the sign of r_2 opposite of r_1 and alpha_2 opposite of alpha_1
            r_2 = -np.sign(r_1)*r_2
            alpha_2 = -np.sign(alpha_1)*alpha_2

            pose_s = Pose2D(x_s, y_s, theta_s)
            [pose_m, pose_f] = self.maneuverPoses(pose_s, r_1, alpha_1, abs(r_2), abs(alpha_2)) # this function expects r_2 and alpha_2 to be positve values

            # Publish first segment of maneuver
            path1 = self.maneuverSegmentPath(pose_s, r_1, alpha_1)
            self.maneuver_path.path.header.stamp = rospy.Time.now()
            self.maneuver_path.path.poses = []
            for i in range(len(path1)):
                point = PoseStamped()
                point.pose.position.x = path1[i][0]
                point.pose.position.y = path1[i][1]
                self.maneuver_path.path.poses.append(point)
                # Set gear, positive alpha = forward gear
                self.maneuver_path.gear = np.sign(alpha_1)
            self.path1_pub.publish(self.maneuver_path)

            # Publish second segment of maneuver
            path2 = self.maneuverSegmentPath(pose_m, r_2, alpha_2)
            self.maneuver_path.path.poses = []
            for i in range(len(path2)):
                point = PoseStamped()
                point.pose.position.x = path2[i][0]
                point.pose.position.y = path2[i][1]
                self.maneuver_path.path.poses.append(point)
                # Set gear, positive alpha = forward gear
                self.maneuver_path.gear = np.sign(alpha_2)
            self.path2_pub.publish(self.maneuver_path)

            self.optimization_success = res.success

            if (self.optimization_success):
                # If optimization was successful, publish the new target
                # position for the A* algorithm (you will want to make this a
                # separate "goal" value distinct from the roll target position)
                rospy.set_param("/control_panel_node/goal_x", float(pose_s.x))
                rospy.set_param("/control_panel_node/goal_y", float(pose_s.y))

                # Publish the starting pose for the approach b-spline path
                obstacle_end_pose = PoseStamped()
                obstacle_end_pose.header.frame_id = "/odom"
                obstacle_end_pose.pose.position.x = pose_f.x
                obstacle_end_pose.pose.position.y = pose_f.y
                quat_forklift = quaternion_from_euler(0, 0, wrapToPi(pose_f.theta))
                obstacle_end_pose.pose.orientation.x = quat_forklift[0]
                obstacle_end_pose.pose.orientation.y = quat_forklift[1]
                obstacle_end_pose.pose.orientation.z = quat_forklift[2]
                obstacle_end_pose.pose.orientation.w = quat_forklift[3]

                self.approach_pose_pub.publish(obstacle_end_pose)

            return OptimizeManeuverResponse(self.optimization_success)

        else:
            return OptimizeManeuverResponse(False)


    def occupancyGridCallback(self, msg):
        '''
        Callback function for the OccupancyGrid created by the mapping node.
        Sets the member variable 'self.obstacles' as a vector of points
        representing the presence of obstacles on the map.

        Args:
        -----
        msg: nav_msgs/OccupancyGrid message

        Returns:
        None
        '''
        self.obstacles = self.gridMapToObstacles(msg)

    def rollCallback(self, msg):
        '''
        Reads in the rolls target pose. The (x,y) position are contained in the
        pose.position variable and the yaw angle is contained int he
        pose.orientation.z variable.

        Args:
        -----
        msg: PoseStamped object containing the roll position in
             msg.pose.position.x, msg.pose.position.y and the yaw angle in
             msg.pose.orientation.z

        Returns:
        --------
        None
        '''
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        euler_angles = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        self.target_approach_angle = euler_angles[2]

    def odomCallback(self, msg):
        '''
        Stores the forklift's current pose from odometry data.

        Args:
        -----
        msg: nav_msgs/Odometry object

        Returns:
        --------
        None
        '''
        self.current_pose = msg.pose.pose

if __name__ == "__main__":
    try:
        maneuver_path = ManeuverPath()
        maneuver_path.spin()
    except rospy.ROSInterruptException:
        pass
