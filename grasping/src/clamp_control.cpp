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
	

public:
	ClampControl() : nh(""), nh_("~")
	{
		// to be subscribed to the controller for operation
		clamp_control_sub = nh.subscribe<std_msgs::Int16> ("controller", 1, &ClampControl::clamp_Callback, this);
	
		limit_switch_up_sub = nh.subscribe<std_msgs::Bool> ("clamp_control/limit_switch_up", 1, &ClampControl::limit_up_Callback, this);
		limit_switch_down_sub = nh.subscribe<std_msgs::Bool> ("clamp_control/limit_switch_down", 1, &ClampControl::limit_down_Callback, this);
		limit_switch_open_sub = nh.subscribe<std_msgs::Bool> ("clamp_control/limit_switch_open", 1, &ClampControl::limit_open_Callback, this);
		limit_switch_close_sub = nh.subscribe<std_msgs::Bool> ("clamp_control/limit_switch_close", 1, &ClampControl::limit_close_Callback, this);
		force_sub = nh.subscribe<std_msgs::Int16> ("clamp_control/force", 1, &ClampControl::force_Callback, this);
		stretch_sub = nh.subscribe<std_msgs::Float32> ("clamp_control/stretch", 1, &ClampControl::stretch_Callback, this);
		
                clamp_movement_pub = nh_.advertise<std_msgs::Float32>("clamp_movement", 1);
                clamp_grasp_pub = nh_.advertise<std_msgs::Float32>("clamp_grasp", 1);
		

	}

	
	void controller();	

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


void ClampControl::controller()
{
	float clamp_movement;
	float clamp_grasp;

	bool clamp_opening;
	bool clamp_closing;
	bool clamp_raising;
	bool clamp_lowering;
	bool clamp_plate;
	bool force_range;
	
	if (limit_open == false)
	{
		clamp_opening = true;
	}

	else if (limit_close == false)
	{
		clamp_closing = true;
	}

	else if (limit_down == false)
	{
		clamp_lowering = true;
	}

	else if (limit_up == false)
	{
		clamp_raising = true;
	}

        else if (stretch > 18.0 && stretch < 22.0)
	{
		clamp_plate == true;
	}

	else if (force < 970)
	{
		force_range == true;
	}

	// Picking operation
	if (controller_value == 7)
	{
		// lower the clamp && limit switch, check for stretch sensor, start grasp, check for fsr && limit switch, lift clamp
		if (clamp_opening == true)
		{
			clamp_grasp = 0.5;
		}

		if (clamp_lowering == true)
		{
			clamp_movement = -0.5;
		}

		if (clamp_plate == true)
		{
			if ((clamp_closing == true) && force_range)
			{
				clamp_grasp = -0.5;
			}
			else
			{
				if (clamp_raising == true)
				{	
					clamp_movement = 0.5;
				}
			}
			
		}
	}

	// Placing operation
	else if (controller_value == 8)
	{
		// lower the clamp && limit switch, open clamp && limit switch, once moved away, lift clamp && limit switch
		if (clamp_lowering == true)
		{
			clamp_movement = -0.5;
		}
		
		if (clamp_opening == true) // open clamp
		{
			clamp_grasp = 0.5;
		}
		else
		{
			clamp_movement = 0.5;
		}
	}

	else
	{
		clamp_movement = 0.0;
		clamp_grasp = 0.0;
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "clamp_control");

	ClampControl clamp_control;

	ros::Rate rate(3);
	while (ros::ok())
	{
		clamp_control.controller();
		ros::spinOnce;
		rate.sleep();
	}
	return 0;
}
