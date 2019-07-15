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

	ros::Publisher clamp_plate_status_pub;
	ros::Publisher grasp_status_pub;
	
	bool limit_up;
	bool limit_down;
	bool limit_open;
	bool limit_close;
	int force;
	float stretch;
	int controller_value;

	float clamp_grasp;
	float clamp_movement;
	bool clamp_plate_status;
	bool grasp_status;
	int operation_mode;
	
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
		
		clamp_plate_status_pub = nh_.advertise<std_msgs::Bool>("clamp_plate_status", 1);
		grasp_status_pub = nh_.advertise<std_msgs::Bool>("grasp_status", 1);

		operation_mode = 0; // starting with raising clamp
	}

	
	void controller();	
	float open_clamp();
	float lower_clamp();
	float raise_clamp();
	float close_clamp();
	bool stretch_check();
	bool check_grasp();

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
	}
	else
	{
		clamp_grasp = 0.0;
	}

	operation_mode = 2;
	return clamp_grasp;
}

float ClampControl::lower_clamp()
{
	if (limit_down == false)
	{
		clamp_movement = 0.5;
	}
	else
	{
		clamp_movement = 0.0;
	}

	operation_mode = 1;
	return clamp_movement;
}

bool ClampControl::stretch_check()
{
	if (stretch > 18.0 && stretch < 22.0)
	{
		clamp_plate_status = true;
	}

	else
	{
		clamp_plate_status = false;
	}

	operation_mode = 3;
	return clamp_grasp;
}

float ClampControl::raise_clamp()
{
	if (limit_up == false)
	{
		clamp_movement = -0.5;
	}
	else
	{
		clamp_movement = 0.0;
	}

	operation_mode = 6;
	return clamp_movement;
}

float ClampControl::close_clamp()
{
	if (limit_close == false)
	{
		clamp_grasp == -0.5;
	}
	else
	{
		clamp_grasp = 0.0;
	}
	
	operation_mode = 4;
	return clamp_grasp;
} 

bool ClampControl::check_grasp()
{
	if ((clamp_plate_status == true) && (force < 962))
	{
		grasp_status = true;
		operation_mode == 5;
	}
	else
	{
		grasp_status = false;
		operation_mode == 1;
	}
	
	return grasp_status;
}

void ClampControl::controller()
{
	// Picking operation
	if (controller_value == 7)
	{
		if (operation_mode == 0)
		{
			lower_clamp();
		}
		if (operation_mode == 1)
		{
			open_clamp();
		}
		if (operation_mode == 2)
		{
			stretch_check();
		}
		if (operation_mode == 3)
		{
			close_clamp();
		}
		if (operation_mode == 4)
		{
			check_grasp();
		}
		if (operation_mode == 5)
		{
			raise_clamp();
		}
		if (operation_mode == 6)
		{
			grasp_status == true;
		}
	}

	else
	{	
		clamp_grasp = 0.0;
		clamp_movement = 0.0;
	}
		
	std_msgs::Float32 clamp_movement_msg;
	std_msgs::Float32 clamp_grasp_msg;		
	std_msgs::Bool clamp_plate_status_msg;
	std_msgs::Bool grasp_status_msg;

	clamp_movement_msg.data = clamp_movement;
	clamp_grasp_msg.data = clamp_grasp;
	clamp_plate_status_msg.data = clamp_plate_status;
	grasp_status_msg.data = grasp_status;
	
	clamp_movement_pub.publish(clamp_movement_msg);
	clamp_grasp_pub.publish(clamp_grasp_msg);
	clamp_plate_status_pub.publish(clamp_plate_status_msg);
	grasp_status_pub.publish(grasp_status_msg);	
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
	}
	return 0;
}
