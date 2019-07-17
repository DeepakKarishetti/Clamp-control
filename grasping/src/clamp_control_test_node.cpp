#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

void movementCallback(const std_msgs::Float32&);
void graspCallback(const std_msgs::Float32&);

double clamp_movement = 0.0;
double clamp_grasp = 0.0;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "clamp_control_test_node");
	ros::NodeHandle nh;
    ros::Subscriber clamp_movement_sub = nh.subscribe("/clamp_control/clamp_movement", 1, movementCallback);
    ros::Subscriber clamp_grasp_sub = nh.subscribe("/clamp_control/clamp_grasp", 1, graspCallback);
	ros::Publisher pub_controller = nh.advertise<std_msgs::Int16> ("controller", 3, true); // latched, meaning it waits for subscribers to send a message
	ros::Publisher pub_limit_up = nh.advertise<std_msgs::Bool> ("limit_switch_up", 3);
	ros::Publisher pub_limit_down = nh.advertise<std_msgs::Bool> ("limit_switch_down", 3);
	ros::Publisher pub_limit_open = nh.advertise<std_msgs::Bool> ("limit_switch_open", 3);
	ros::Publisher pub_limit_close = nh.advertise<std_msgs::Bool> ("limit_switch_close", 3);

	ros::Publisher pub_force = nh.advertise<std_msgs::Int16> ("force", 1);
	ros::Publisher pub_stretch = nh.advertise<std_msgs::Float32> ("stretch", 1);

    std_msgs::Int16 msg_controller;

    std_msgs::Int16 msg_force;
    std_msgs::Float32 msg_stretch;

    std_msgs::Bool msg_limit_up;
    std_msgs::Bool msg_limit_down;
    std_msgs::Bool msg_limit_open;
    std_msgs::Bool msg_limit_close;

	ros::Rate rate(30);

    // Publish controller mode
    std::cout << "Publishing controller value\n";
    msg_controller.data = 7;
    pub_controller.publish(msg_controller);

	while (ros::ok())
	{

        std::cout << "clamp_movement: " << clamp_movement << "\n";
        std::cout << "clamp_grasp: " << clamp_grasp << "\n";
        // Clamp Height
        // if clamp_movement is negative, delay and publish limit_up
        if (clamp_movement < 0) {
            std::cout << "Going up\n";
            ros::Duration(1.0).sleep();
            msg_limit_down.data = false;
            pub_limit_down.publish(msg_limit_down);
            msg_limit_up.data = true;
            pub_limit_up.publish(msg_limit_up);
        }
        // if clamp_movement is positive, delay and publish limit_down
        if (clamp_movement > 0) {
            std::cout << "Going down\n";
            ros::Duration(1.0).sleep();
            msg_limit_up.data = false;
            pub_limit_up.publish(msg_limit_up);
            msg_limit_down.data = true;
            pub_limit_down.publish(msg_limit_down);
        }

        // Clamp Grasp
        // if clamp_grasp is negative, delay and publish limit_close
        if (clamp_grasp < 0) {
            std::cout << "Closing\n";
            ros::Duration(1.0).sleep();
            msg_limit_open.data = false;
            pub_limit_open.publish(msg_limit_open);
            msg_limit_close.data = true;
            pub_limit_close.publish(msg_limit_close);
        }
        // if clamp_grasp is positive, delay and publish limit_open
        if (clamp_grasp > 0) {
            std::cout << "Opening\n";
            ros::Duration(1.0).sleep();
            msg_limit_close.data = false;
            pub_limit_close.publish(msg_limit_close);
            msg_limit_open.data = true;
            pub_limit_open.publish(msg_limit_open);
        }

        ros::spinOnce();
		rate.sleep();
	}
}

void movementCallback(const std_msgs::Float32& msg)
{
    clamp_movement = msg.data;
}

void graspCallback(const std_msgs::Float32& msg)
{
    clamp_grasp = msg.data;
}
