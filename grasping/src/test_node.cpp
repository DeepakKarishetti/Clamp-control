#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_node");
	ros::NodeHandle nh;
	ros::Publisher pub_controller = nh.advertise<std_msgs::Int16> ("controller", 1);
	
	ros::Publisher pub_limit_up = nh.advertise<std_msgs::Bool> ("limit_switch_up", 1);
	ros::Publisher pub_limit_down = nh.advertise<std_msgs::Bool> ("limit_switch_down", 1);
	ros::Publisher pub_limit_open = nh.advertise<std_msgs::Bool> ("limit_switch_open", 1);
	ros::Publisher pub_limit_close = nh.advertise<std_msgs::Bool> ("limit_switch_close", 1);

	ros::Publisher pub_force = nh.advertise<std_msgs::Int16> ("force", 1);
	ros::Publisher pub_stretch = nh.advertise<std_msgs::Float32> ("stretch", 1);


	ros::Rate rate(3);
	while (ros::ok())
	{
		std_msgs::Int16 msg_controller;
		
		std_msgs::Int16 msg_force;
		std_msgs::Float32 msg_stretch;

		std_msgs::Bool msg_limit_up;
		std_msgs::Bool msg_limit_down;
		std_msgs::Bool msg_limit_open;
		std_msgs::Bool msg_limit_close;


		msg_controller.data = 8;
		pub_controller.publish(msg_controller);

		msg_force.data = 3950;
		pub_force.publish(msg_force);

		msg_stretch.data = 219.0;
		pub_stretch.publish(msg_stretch);

		msg_limit_up.data = true;
		pub_limit_up.publish(msg_limit_up);

		msg_limit_down.data = true;
		pub_limit_down.publish(msg_limit_down);

		msg_limit_open.data = true;
		pub_limit_open.publish(msg_limit_open);

		msg_limit_close.data = true;
		pub_limit_close.publish(msg_limit_close);

		rate.sleep();
	}
}
