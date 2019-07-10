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
	ros::Subscriber clamp_switch_sub;

        ros::Publisher clamp_movement_pub;
        ros::Publisher clamp_grasp_pub;
	
	int clamp_switch;
	bool limit_up;
	bool limit_down;
	bool limit_open;
	bool limit_close;
	int force;
	float stretch;

public:
	ClampControl() : nh(""), nh_("~")
	{
		clamp_control_sub = nh.subscribe<std_msgs::Float32> ("control", 1, &ClampControl::control_Callback, this);

		// main controller to be subscribed to	

		clamp_switch_sub = nh.subscribe<std_msgs::Int16> ("clamp_switch", 1, &ClampControl::switch_Callback, this);	
	
		limit_switch_up_sub = nh.subscribe<std_msgs::Bool> ("limit_switch_up", 1, &ClampControl::limit_up_Callback, this);
		limit_switch_down_sub = nh.subscribe<std_msgs::Bool> ("limit_switch_down", 1, &ClampControl::limit_down_Callback, this);
		limit_switch_open_sub = nh.subscribe<std_msgs::Bool> ("limit_switch_open", 1, &ClampControl::limit_open_Callback, this);
		limit_switch_close_sub = nh.subscribe<std_msgs::Bool> ("limit_switch_close", 1, &ClampControl::limit_close_Callback, this);
		force_sub = nh.subscribe<std_msgs::Int16> ("force", 1, &ClampControl::force_Callback, this);
		stretch_sub = nh.subscribe<std_msgs::Float32> ("stretch", 1, &ClampControl::stretch_Callback, this);
		
                clamp_movement_pub = nh_.advertise<std_msgs::Float32>("clamp_movement", 1);
                clamp_grasp_pub = nh_.advertise<std_msgs::Float32>("clamp_grasp", 1);
	}


	void switch_Callback(const std_msgs::Int16::ConstPtr& msg)
	{
	 	clamp_switch = msg -> data;
	}	

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

	void control_Callback(const std_msgs::Float32::ConstPtr& msg)
	{
		std_msgs::Float32 clamp_movement;
		std_msgs::Float32 clamp_grasp;
	
		// Picking operation
		if (clamp_switch == 7)
		{
			// lower the clamp && limit switch, check for stretch sensor, start grasp, check for fsr && limit switch, lift clamp
			while (limit_open == false)
			{
				clamp_grasp.data = 0.5;
				clamp_grasp_pub.publish(clamp_grasp); 	
			}

			while (limit_down == false)
			{
				clamp_movement.data = -0.5;
				clamp_movement_pub.publish(clamp_movement);
			}

			while (stretch > 18.0 && stretch < 22.0)
			{
				if ((limit_close == false) && force < 950)
				{
					clamp_grasp.data = -0.5;
					clamp_grasp_pub.publish(clamp_grasp);
				}
				else
				{
					while (limit_up == false)
					{
						clamp_movement.data = 0.5;
						clamp_movement_pub.publish(clamp_movement);
					}
				}
				
			}
		}

		// Placing operation
		else if (clamp_switch == 8)
		{
			// lower the clamp && limit switch, open clamp && limit switch, once moved away, lift clamp && limit switch
			while (limit_down == false)
			{
				clamp_movement.data = -0.5;
				clamp_movement_pub.publish(clamp_movement);
			}
			
			if (limit_open == false)
			{
				clamp_grasp.data = 0.5;
				clamp_grasp_pub.publish(clamp_grasp);
			}
			else
			{
				clamp_movement.data = 0.5;
				clamp_movement_pub.publish(clamp_movement);
			}
		}

		else
		{
			clamp_movement.data = 0.0;
			clamp_movement_pub.publish(clamp_movement);
			clamp_grasp.data = 0.0;
			clamp_grasp_pub.publish(clamp_grasp);
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "clamp_control");

	ClampControl clamp_control;

	ros::Rate rate(3);
	while (ros::ok())
	{
		//	


		rate.sleep();
		ros::spinOnce;
	}
	return 0;
}
