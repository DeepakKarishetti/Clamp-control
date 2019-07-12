#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>

class ClampControl
{
private:
	ros::NodeHandle nh;
        ros::NodeHandle nh_;

        ros::Subscriber clamp_control_sub;

	ros::Subscriber limit_switch_up_sub;
	ros::Subscriber limit_switch_down_sub;
	ros::Subscriber limit_switch_open_sub;
	ros::Subscriber limit_switch_close_sub;
	ros::Subscriber force_sub;
	ros::Subscriber stretch_sub;

        ros::Publisher clamp_movement_pub;
        ros::Publisher clamp_grasp_pub;
	
	bool limit_up;
	bool limit_down;
	bool limit_open;
	bool limit_close;
	int force;
	float stretch;
	int controller_value;

	float clamp_grasp;
	float clamp_movement;
	
public:
	ClampControl() : nh(""), nh_("~")
	{
		// to be subscribed to the controller for operation
		clamp_control_sub = nh.subscribe<std_msgs::Int16> ("controller", 1, &ClampControl::clamp_Callback, this);
	
		limit_switch_up_sub = nh.subscribe<std_msgs::Bool> ("limit_switch_up", 1, &ClampControl::limit_up_Callback, this);
		limit_switch_down_sub = nh.subscribe<std_msgs::Bool> ("limit_switch_down", 1, &ClampControl::limit_down_Callback, this);
		limit_switch_open_sub = nh.subscribe<std_msgs::Bool> ("limit_switch_open", 1, &ClampControl::limit_open_Callback, this);
		limit_switch_close_sub = nh.subscribe<std_msgs::Bool> ("limit_switch_close", 1, &ClampControl::limit_close_Callback, this);
		force_sub = nh.subscribe<std_msgs::Int16> ("force", 1, &ClampControl::force_Callback, this);
		stretch_sub = nh.subscribe<std_msgs::Float32> ("stretch", 1, &ClampControl::stretch_Callback, this);
		
                clamp_movement_pub = nh_.advertise<std_msgs::Float32>("clamp_movement", 1);
                clamp_grasp_pub = nh_.advertise<std_msgs::Float32>("clamp_grasp", 1);
		

	}

	
	void controller();	
	float open_clamp();
	float lower_clamp();
	float raise_clamp();
	float close_clamp();
	float grasp();

	void limit_up_Callback(const std_msgs::Bool::ConstPtr& msg)
	{
		limit_up = msg -> data;
	}

	void limit_down_Callback(const std_msgs::Bool::ConstPtr& msg)
        {
                limit_down = msg -> data;
        }

	void limit_open_Callback(const std_msgs::Bool::ConstPtr& msg)
        {
                limit_open = msg -> data;
        }

	void limit_close_Callback(const std_msgs::Bool::ConstPtr& msg)
        {
                limit_close = msg -> data;
        }

	void force_Callback(const std_msgs::Int16::ConstPtr& msg)
	{
		force = msg -> data;
	}
	
	void stretch_Callback(const std_msgs::Float32::ConstPtr& msg)
	{
		stretch = msg -> data;
	}

	void clamp_Callback(const std_msgs::Int16::ConstPtr& msg)
	{
			controller_value = msg -> data;
	}	
};

float ClampControl::open_clamp()
{
	if (limit_open == false)
	{
		clamp_grasp = 0.5;
		//ros::spinOnce();
	}
	else
	{
		clamp_grasp = 0.0;
	}
	return clamp_grasp;
}

float ClampControl::lower_clamp()
{
	if (limit_down == false)
	{
		clamp_movement = -0.5;
		//ros::spinOnce();
	}
	else
	{
		clamp_movement = 0.0;
	}

	return clamp_movement;
}

float ClampControl::grasp()
{
	if (stretch > 18.0 && stretch < 22.0)
	{
		if ((limit_close == false) && force < 962)
		{
			clamp_grasp = -0.5;
			//ros::spinOnce();
		}
		else
		{
			clamp_grasp = 0.0;
		}
	}
	else
	{
		clamp_grasp = 0.0;
	}

	return clamp_grasp;
}

float ClampControl::raise_clamp()
{
	if (limit_up == false)
	{
		clamp_movement = 0.5;
		//ros::spinOnce();
	}
	else
	{
		clamp_movement = 0.0;
	}

	return clamp_movement;
}

float ClampControl::close_clamp()
{
	if (limit_close == false)
	{
		clamp_grasp == -0.5;
		//ros::spinOnce();
	}
	else
	{
		clamp_grasp = 0.0;
	}
	
	return clamp_grasp;
} 


void ClampControl::controller()
{

	if (controller_value == 7)
	{
		if (open_clamp())
		{	
			if (lower_clamp())
			{
				if (grasp())
				{
					raise_clamp();
				}
			}
		}
	}
	
	else if (controller_value == 8)
	{
		if (lower_clamp())
		{
			if (open_clamp())
			{
				raise_clamp();
			}
		}
	}

	else
	{	
		clamp_grasp = 0.0;
		clamp_movement = 0.0;
	}
		
	std_msgs::Float32 clamp_movement_msg;
	std_msgs::Float32 clamp_grasp_msg;		

	clamp_movement_msg.data = clamp_movement;
	clamp_grasp_msg.data = clamp_grasp;

	clamp_movement_pub.publish(clamp_movement_msg);
	clamp_grasp_pub.publish(clamp_grasp_msg);	
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "clamp_control");

	ClampControl clamp_control;

	ros::Rate rate(3);
	while (ros::ok())
	{
		clamp_control.controller();
		ros::spinOnce();
		rate.sleep();
		//ros::MultiThreadedSpinner spinner(4);
		//spinner.spin();
	}
	return 0;
}
