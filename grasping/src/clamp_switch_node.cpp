#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Joy.h>

class ClampSwitch
{
private:
	ros::NodeHandle nh;
	ros::NodeHandle nh_;
	ros::Subscriber joy_sub;
	ros::Publisher clamp_switch_pub;
	ros::Publisher clamp_movement_pub;
	ros::Publisher clamp_grasp_pub;

	int deadman_button;
	double scale_linear;

public:
	ClampSwitch() : nh(""), nh_("~")
	{
		joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &ClampSwitch::joyCallback, this);
		clamp_switch_pub = nh_.advertise<std_msgs::Bool>("clamp_switch", 1);
		clamp_movement_pub = nh_.advertise<std_msgs::Float32>("clamp_movement", 1);
		clamp_grasp_pub = nh_.advertise<std_msgs::Float32>("clamp_grasp", 1);	

		nh.param<int>("deadman", deadman_button, 4);
		nh.param<double>("scale_linear", scale_linear, 1.0);
	}

	void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
	{
		std_msgs::Bool clamp_switch;
		std_msgs::Float32 clamp_movement;
		std_msgs::Float32 clamp_grasp;
		//
		//
		if (msg->buttons[deadman_button] == 0)
		{
			clamp_switch.data = false;
			clamp_switch_pub.publish(clamp_switch);

			clamp_movement.data = 0;
			clamp_movement_pub.publish(std_msgs::Float32(clamp_movement));

			clamp_grasp.data = 0;
			clamp_grasp_pub.publish(std_msgs::Float32(clamp_grasp));

		}
		else {
			clamp_switch.data = true;
			clamp_switch_pub.publish(clamp_switch);

			clamp_movement.data = msg->axes[1];
			clamp_movement.data = std::min(1.0, static_cast<double>(clamp_movement.data));
			clamp_movement.data = std::max(-1.0, static_cast<double>(clamp_movement.data));
			
			clamp_movement.data *= scale_linear;
			clamp_movement_pub.publish(clamp_movement);

			clamp_grasp.data = msg->axes[0];
			clamp_grasp.data = std::min(1.0, static_cast<double>(clamp_grasp.data));
			clamp_grasp.data = std::max(-1.0, static_cast<double>(clamp_grasp.data));

			clamp_grasp.data *= scale_linear;
			clamp_grasp_pub.publish(clamp_grasp);
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "clamp_switch_node");

	ClampSwitch clamp_switch_node;

	ros::spin();
	return 0;
}
