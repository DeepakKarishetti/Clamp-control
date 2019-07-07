'''
Module containing methods for determining if a point is within a polygon
represented by a vector of points.
'''


import numpy as np
import copy


def isOdd(x):
    if (x % 2 == 0):
        return False
    else:
        return True

def rayIntersectsSegment(point_in, segment):
    '''
    Determines whether a given point with a ray shooting in the positive X
    direction intersects a line segment specifiec by two points. Returns 0 if it
    does not intersect, 1 if it does intersect, 2 if it is on the segment.

    Args:
    -----
    point_in: 2x1 numpy array containing an (x,y) position
    segment: 2x2 numpy array containing 2 (x,y) points that define a line
             segment

    Returns:
    --------
    intersect: 0 if no intersection, 1 if intersects but not on the segment, 2
               if on the segment

    Diagrams

              Bad
               ^
               |
    --------------o
           |     /T
       G   |  ? / |
       o   |   /  | -> Bad
       o   |  / ? |
       d   |B/    |
    -------o-------
               |
               v
              Bad

      point   o
        o    /T
       /   /
      / \/ point angle
     / /  \
    // \   |
    o----------
    B  segment angle
    '''
    # If the point is equal to one of the vertices, return 2 for "on the segment"
    point = copy.copy(point_in) # this keeps "point_in" from being altered

    if ((point[0] == segment[0,0] and point[1] == segment[0,1]) or (point[0] == segment[1,0] and point[1] == segment[1,1])):
        return 2

    # If the point is equal with one of the points in the Y dimension, increase
    # its Y position by a small amount
    if (point[1] == segment[0,1] or point[1] == segment[1,1]):
        point[1] += 0.000001

    # Find the top and bottom points in the Y direction
    if (segment[0,1] > segment[1,1]):
        top = segment[0,:]
        bottom = segment[1,:]
    else:
        top = segment[1,:]
        bottom = segment[0,:]

    # Get min and max values
    x_max = max(segment[0,0], segment[1,0])
    x_min = min(segment[0,0], segment[1,0])
    y_max = top[1]
    y_min = bottom[1]

    # Check if point is in the "Bad" region
    if (point[0] > x_max or point[1] < y_min or point[1] > y_max):
        return 0
    # Check if point is in the "Good" region
    elif (point[0] < x_min):
        return 1
    # Check if point is in the "?" region
    else:
        # Calculate the angle of the segment and the angle of the point with the
        # segment's bottom point. If the point angle is larger than the segment
        # angle, then the ray crosses the segment. If they are equal, the point
        # is on the segment. If the point angle is smaller, the ray does not
        # cross.
        segment_angle = np.arctan2(top[1] - bottom[1], top[0] - bottom[0])
        point_angle = np.arctan2(point[1] - bottom[1], point[0] - bottom[0])

        # Check angle between point, bottom of segment, and positive X direction
        if (point_angle > segment_angle):
            return 1
        elif (point_angle < segment_angle):
            return 0
        else:
            return 2

def pointInPolygon(points, polygon):
    '''
    Iterates through a vector of points and determines whether the point is
    within the polygon specified by the vector of points in "polygon". The
    polygon is assumed to be a "simple polygon" (there are no intersections).
    The first returned vector is of the same length as "points" and contains 0
    if the associated point is out of the polygon and 1 if the point is in the
    polygon or on the edge. The second optional output vector indicates whether
    the point is specifically on the edge of the polygon.

    Args:
    -----
    points: mx2 vector of points to be checked if they are in a polygon
    polygon: nx2 vector of vertices of a polygon (must be a simple polygon and
             cannot intersect)

    Returns:
    --------
    in_poly: mx1 vector indicating whether each point in "points" is in the
             polygon or not
    on_poly: mx1 vector indicating whether each point in "points" is on the
             polygon edge or not
    '''
    # Handle the single point case where there is only one dimension or the dimensions are flipped. This includes: (2,), (2,1)
    if (points is None):
        return np.zeros([1,1]), np.zeros([1,1])
    elif (len(points.shape) == 1):
        points = points.reshape(1,2)
    elif (points.shape[1] != 2):
        points = points.reshape(1,2)

    # Get size of points to check, should be length of first dimension
    num_points = points.shape[0]
    num_polygon_points = polygon.shape[0]
    in_poly = np.zeros([num_points, 1])
    on_poly = np.zeros([num_points, 1])

    # Iterate through all the points and determine if they are in the polygon
    #for i in range(num_points):
    for i in range(num_points):
        # initialize crossing count to 0
        # if the number of crossings is odd, the point is inside, if even,
        # it is outside.
        count = 0
        for j in range(num_polygon_points - 1):
            segment = np.array([polygon[j,:], polygon[j+1,:]])
            intersect = rayIntersectsSegment(points[i,:],segment)

            if (intersect == 1):
                # Segment is crossed
                count += 1
            elif (intersect == 2):
                # Point is on the segment, if on the segment it must be in the
                # polygon, move to next point
                in_poly[i] = 1
                on_poly[i] = 1
                break

        # If number of counts is odd, the point is inside, if even it is outside
        if (isOdd(count)):
            in_poly[i] = 1

    return in_poly, on_poly
