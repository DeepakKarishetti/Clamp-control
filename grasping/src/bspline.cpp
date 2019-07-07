/**
 * Generates a B-spline curve from a set of control points. The spline in this
 * code is intended to be cubic (p = 3) but can be changed to any order as long
 * as you have enough control points. The B-spline knot vector is defined wiht
 * the range [0,1] and uniformly split. The vector of points along the line
 * (defined as 'x') is also on the range of [0,1] and split uniformly, but the
 * resolution of the line can be defined. The path is generated using De Boor's
 * Algorithm (https://en.wikipedia.org/wiki/De_Boor%27s_algorithm) and then
 * converted to a ROS nav_msgs:Path message and published.
 */

// TODO: consider making your own message type with a vector of
// geometry_msgs::Points[] as the control points, an Int as the polynomial
// order, and another Int for the line resolution

// TODO: add a parameter that let's you set the frame that the path is in
// (odom/map/world/etc)

// TODO: publish an array of marker points for visualizing the control points,
// make this a debugging feature that you can turn on or off

#include <iostream>
#include <vector>
#include <cmath> // used for M_PI (pi constant)
#include <Eigen>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace Eigen;

class GraspPath
{
private:
    // Path Variables
    vector<Vector2d> m_control_points; // Control points
    vector<double> m_knots; // Knot vector
    vector<double> m_x; // 'x' vector
    int m_x_size; // path resolution
    vector<int> m_k; // 'k' vector
    int m_p; // Polynomial order
    vector<Vector2d> m_path; // Vector path

    // ROS Objects
    ros::NodeHandle nh_;
    ros::Subscriber sub_roll; // reads the roll position and approach angle
    ros::Subscriber sub_forklift; // reads the current forklift pose
    ros::Publisher pub_path;
    bool debug_control_points; // if true, publishes the control points as markers for rviz
    ros::Publisher pub_control_points;
    nav_msgs::Path ros_path;
    visualization_msgs::Marker ros_control_points;
    geometry_msgs::PoseStamped roll_pose; // contains the roll position and approach angle
    geometry_msgs::PoseStamped forklift_pose; // the forklifts current pose

    // Forklift Dimensions
    double body_length; // forklift body length
    double clamp_length; // length of long arm of clamp
    double total_length; // total length of the forklift + clamp

    // Roll parameters
    double roll_radius; // radius of the paper roll

public:
    GraspPath() : nh_("~")
    {
        // Store parameters
        nh_.param<int>("path/polynomial_order", m_p, 3); // Define the polynomial order (prefer cubic, p = 3)
        nh_.param<int>("path/resolution", m_x_size, 20); // Define resolution of the line
        nh_.param<double>("/forklift/body/length", body_length, 2.5601);
        nh_.param<double>("/forklift/body/total", total_length, 3.5659);
        nh_.param<double>("/forklift/clamp/long_length", clamp_length, 1.0058);
        nh_.param<double>("roll_radius", roll_radius, 0.20);

        // Set up control points
        m_control_points.push_back(Vector2d(1,1));
        m_control_points.push_back(Vector2d(2,5));
        m_control_points.push_back(Vector2d(4,7));
        m_control_points.push_back(Vector2d(6,8));
        m_control_points.push_back(Vector2d(10,9));
        m_control_points.push_back(Vector2d(9,6));
        m_control_points.push_back(Vector2d(8,4));
        m_control_points.push_back(Vector2d(6,3));
        m_control_points.push_back(Vector2d(5,2.5));
        m_control_points.push_back(Vector2d(4,3));

        // Define ROS Objects
        pub_path = nh_.advertise<nav_msgs::Path>("path", 1);
        // NOTE: the pose information from this message only contains the (x,y)
        // location of the roll and the desired approach angle stored in the
        // variable 'pose.pose.orientation.z'. The orientation is not an actual
        // quaternion in this particular case.
        cout << "[WARN]: The pose sent to the '/bspline_path/roll/pose' topic constains the 2D position in (pose.pose.position.x, pose.pose.position.y) and the desired approach angle as (pose.pose.orientation.z)\n";
        sub_roll = nh_.subscribe("roll/pose", 1, &GraspPath::rollCallback, this);
        sub_forklift = nh_.subscribe("/odom", 1, &GraspPath::forkliftCallback, this);

        // Publish the control points for debugging visualization
        debug_control_points = true;
        if (debug_control_points) {
            // Create publisher
            pub_control_points = nh_.advertise<visualization_msgs::Marker>("control_points", 1);

            // Initialize Marker message
            ros_control_points.header.frame_id = "odom";
            ros_control_points.id = 0;
            ros_control_points.type = visualization_msgs::Marker::SPHERE_LIST;
            ros_control_points.action = visualization_msgs::Marker::ADD;
            ros_control_points.scale.x = 0.1;
            ros_control_points.scale.y = 0.1;
            ros_control_points.scale.z = 0.1;
            ros_control_points.color.r = 1;
            ros_control_points.color.g = 0;
            ros_control_points.color.b = 0;
            ros_control_points.color.a = 1;
            ros_control_points.lifetime = ros::Duration(0);

            ros_control_points.pose.position.x = 0;
            ros_control_points.pose.position.y = 0;
            ros_control_points.pose.position.z = 0;
            ros_control_points.pose.orientation.x = 0;
            ros_control_points.pose.orientation.y = 0;
            ros_control_points.pose.orientation.z = 0;
            ros_control_points.pose.orientation.w = 1;

            updateControlPoints();
        }

        constructVectors();
        generatePath();
    }

    // De Boor's Algorithm Implementation
    Vector2d deBoors(int k, double x, const vector<double> &t, const vector<Vector2d> &c, int p)
    {
        /**
         * Evaluates the B-spline function at 'x'
         *
         * Args
         * -----
         * k: index of knot interval that contains x
         * x: position along curve (goes between lowest knot value to highest
         *    knot value)
         * t: array of knot positions, needs to be padded with 'p' extra
         *    endpoints
         * c: array of control points
         * p: degree of B-spline
         */

        vector< Vector2d > d(p+1);
        for (int j = 0; j <= p; j++) {
            d.at(j) = c.at(j+k-p);
        }
        for (int r = 1; r <= p; r++) {
            for (int j = p; j >= r; j--) {
                double alpha = (x - t.at(j+k-p)) / (t.at(j+1+k-r) - t.at(j+k-p));
                d.at(j) = (1 - alpha)*d.at(j-1) + alpha*d.at(j);
            }
        }

        return d.at(p);
    }

    // Generate vectors using the given control points
    void constructVectors()
    {
        /**
         * Generates the knot vector and the vectors for the position along the
         * path (m_x) and its corresponding knot vector interval (m_k). A check
         * is made to make sure the polynomial order does not exceed the number
         * of control points - 1.
         */

        // Check that the polynomial order does not exceed control points - 1
        if (m_p >= m_control_points.size()) {
            // DEBUG: print a warning
            cout << "[WARN]: The provided B-spline polynomial order of " << m_p << " requires at least " << (m_p + 1) << " control points.";
            cout << " Only " << m_control_points.size() << " control points were provided.";
            cout << " The polynomial order will be reduced to " << (m_control_points.size() - 1) << ".\n";

            m_p = m_control_points.size() - 1;
        }

        // Generate the knot vector based on the number of control points
        // Need at least (m_p - 3) more control points than internal knot points
        // (that's m_p - 1 more control points than internal knots + the end points, so you need to add 'm_p' repeated knot endpoints)
        int m_knot_size = m_control_points.size() - (m_p - 1);
        m_knots.resize(m_knot_size, 0.0);
        double div_size = 1.0/(m_knot_size - 1.0);
        for (int i = 0; i < m_knots.size(); i++) {
            m_knots.at(i) = i*div_size;
        }

        // Append the additional 'm_p' knots at each end
        m_knots.resize(m_knot_size + m_p, 1.0);
        m_knots.insert(m_knots.begin(), m_p, 0.0);

        // Generate the 'm_x' and 'm_k' vectors
        m_x.resize(m_x_size, 0.0);
        m_k.resize(m_x_size, 0);
        div_size = 1.0/(m_x_size - 1.0);
        for (int i = 0; i < m_x.size(); i++) {
            m_x.at(i) = i*div_size;
            for (int j = m_p; j < (m_knots.size()-1)-m_p; j++) {
                if (m_x.at(i) >= m_knots.at(j)) {
                    m_k.at(i) = j;
                }
            }
        }
    }

    // Iterate through all positions using DeBoor's algorithm to generate the path vector
    void generatePath()
    {
        /**
         * Iterates through each point in the 'm_x' vector and calculates the
         * corresponding path position using De Boor's algorithm.
         */
        m_path.resize(m_x.size());

        for (int i = 0; i < m_x.size(); i++) {
            Vector2d path_point = deBoors(m_k.at(i), m_x.at(i), m_knots, m_control_points, m_p);
            m_path.at(i) = path_point;
        }

        // Convert vector path into a ROS message path
        ros_path.header.frame_id = "odom";
        ros_path.poses.resize(m_path.size());
        geometry_msgs::PoseStamped pose;
        int path_seq = 0;
        for (int i = 0; i < m_path.size(); i++) {
            pose.pose.position.x = m_path.at(i)[0];
            pose.pose.position.y = m_path.at(i)[1];
            pose.header.seq = path_seq++;
            ros_path.poses.at(i) = pose;
        }
    }

    // This callback generates the desired path and publishes it once
    void rollCallback(const geometry_msgs::PoseStamped msg)
    {
        /**
         * This callback function receives the pose data for the paper roll and
         * calculates the B-spline curve from the forklifts current position to
         * the roll at the desired approach angle. The approach angle is
         * contained in the 'z' component of the 'orientation' part of the
         * 'pose'.
         *
         * The control points are calculated using the roll and forklift poses
         * considering the desired approach angle and stopping point. There are
         * four points placed coming out from the roll in the direction of the
         * approach angle. One on the roll surface. One a clamp's length away.
         * Once two clamp's lengths away. And one a forklift length way from
         * the second point. This is so the baselink, which is the middle of
         * the front axle, will be roughly two clamp lengths away from the roll
         * while being aligned straight with the approach angle. This way the
         * tip of the clamp should be a full clamp length away and the
         * fine-tuned roll detection can be used for the final approach
         * distance.
         */
        // Update the roll pose
        roll_pose = msg;
        double x_r = roll_pose.pose.position.x; // x location of roll
        double y_r = roll_pose.pose.position.y; // y location of roll
        double alpha = roll_pose.pose.orientation.z; // approach angle
        double x_f = forklift_pose.pose.position.x; // x location of forklift
        double y_f = forklift_pose.pose.position.y; // y location of forklift

        // Get yaw angle from quaternion
        tf::Quaternion q(
            forklift_pose.pose.orientation.x,
            forklift_pose.pose.orientation.y,
            forklift_pose.pose.orientation.z,
            forklift_pose.pose.orientation.w
        );
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        // Because the forklift will be driving in the reverse direction, the
        // raw yaw angle must be flipped by 180 deg to point the path in the
        // right direction. The controller will publish negative values for the
        // velocity to account for this.
        yaw = yaw + M_PI;

        // Calculate the control points for the desired path
        m_control_points.clear();
        m_control_points.push_back(Vector2d(x_f, y_f));
        m_control_points.push_back(Vector2d(clamp_length*cos(yaw) + x_f, clamp_length*sin(yaw) + y_f));
        m_control_points.push_back(Vector2d((roll_radius + clamp_length + total_length)*cos(alpha) + x_r, (roll_radius + clamp_length + total_length)*sin(alpha) + y_r));
        m_control_points.push_back(Vector2d((roll_radius + 2*clamp_length)*cos(alpha) + x_r, (roll_radius + 2*clamp_length)*sin(alpha) + y_r));
        m_control_points.push_back(Vector2d((roll_radius + clamp_length)*cos(alpha) + x_r, (roll_radius + clamp_length)*sin(alpha) + y_r));
        m_control_points.push_back(Vector2d(roll_radius*cos(alpha) + x_r, roll_radius*sin(alpha) + y_r));

        updateControlPoints();
        publishControlPoints();
        constructVectors();
        generatePath();
        publishPath();
    }

    void forkliftCallback(const nav_msgs::Odometry msg)
    {
        // Update the forklift pose
        forklift_pose.header = msg.header;
        forklift_pose.pose = msg.pose.pose;
    }

    // Publish
    void publishPath()
    {
        pub_path.publish(ros_path);
    }

    // Convert control points vector into marker points
    void updateControlPoints()
    {
        /**
         * The control points used to generate the B-spline curve are stored in
         * 'm_control_points' as 'Vector2d' values. These points are converted
         * into a set of markers for a ROS message to visualize in RVIZ.
         */
        ros_control_points.points.resize(m_control_points.size());
        for (int i = 0; i < m_control_points.size(); i++) {
            ros_control_points.points.at(i).x = m_control_points.at(i)[0];
            ros_control_points.points.at(i).y = m_control_points.at(i)[1];
        }
    }

    // Publish control points as markers for debugging
    void publishControlPoints()
    {
        if (debug_control_points) {
            // // Delete previous markers
            // visualization_msgs::Marker delete_markers;
            // delete_markers.action = visualization_msgs::Marker::DELETEALL;
            // Publish the values
            pub_control_points.publish(ros_control_points);
        }
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "bspline_path");
    GraspPath grasp_path = GraspPath();

    ros::Rate rate(10);

    while (ros::ok()) {
        grasp_path.publishPath();
        //grasp_path.publishControlPoints();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
