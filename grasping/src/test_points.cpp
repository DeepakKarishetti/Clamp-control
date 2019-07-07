/**
 * This node generates the test points for the forklift to create the B-spline
 * path for testing the grasping approach controls.
 */


#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>


int main(int argc, char** argv)
{
    // Forklift and Roll Parameters
    double roll_x = 0;
    double roll_y = 6;
    double approach_angle = (5.0/4.0)*M_PI;
    double forklift_x = 2;
    double forklift_y = 0;
    double forklift_yaw = 0;

    // Start ROS node
    ros::init(argc, argv, "test_points");
    ros::NodeHandle nh_("~");
    ros::Publisher forklift_pub = nh_.advertise<nav_msgs::Odometry>("/odom", 1);
    ros::Publisher roll_pub = nh_.advertise<geometry_msgs::PoseStamped>("/bspline_path/roll/pose", 1);

    // Register the publishers before proceeding
    // This spot delays 1 second to give the publishers time to register before
    // actually sending the message.
    ros::Rate rate(1);
    rate.sleep();
    ros::spinOnce();


    // Publish the forklift position first
    nav_msgs::Odometry forklift_odom;
    forklift_odom.header.frame_id = "/odom";
    forklift_odom.header.stamp = ros::Time::now();
    forklift_odom.pose.pose.position.x = forklift_x;
    forklift_odom.pose.pose.position.y = forklift_y;

    // Get quaternion orientation from the given Yaw angle
    tf::Matrix3x3 m;
    m.setEulerYPR(forklift_yaw, 0, 0);
    tf::Quaternion q;
    m.getRotation(q);
    forklift_odom.pose.pose.orientation.x = q.x();
    forklift_odom.pose.pose.orientation.y = q.y();
    forklift_odom.pose.pose.orientation.z = q.z();
    forklift_odom.pose.pose.orientation.w = q.w();

    // Publish forklift pose
    forklift_pub.publish(forklift_odom);

    // Then publish the roll position to generate the B-spline path
    geometry_msgs::PoseStamped roll_pose;
    roll_pose.header.frame_id = "/odom";
    roll_pose.header.stamp = ros::Time::now();
    roll_pose.pose.position.x = roll_x;
    roll_pose.pose.position.y = roll_y;
    roll_pose.pose.orientation.z = approach_angle;

    // Publish roll pose
    roll_pub.publish(roll_pose);

    ros::spin();

    return 0;
}
