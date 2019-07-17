#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <signal.h>
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
    ros::Publisher grasp_finished_pub;

	bool limit_up;
	bool limit_down;
	bool limit_open;
	bool limit_close;
	int force;
	float stretch;
	int controller_value;

    double clamp_scale; // adjusts value sent to the arduino controlling the clamp, range: [0, 1]
	float clamp_grasp;
	float clamp_movement;
	bool clamp_plate_status;
	bool grasp_status;
	int operation_mode;

    int window_size = 10;
    std::vector<float> stretch_window;

public:
	ClampControl() : nh(""), nh_("~")
	{
        // Set parameters
        nh_.param<double>("clamp_scale", clamp_scale, 0.5);

        std::cout << "Clamp scale: " << clamp_scale << '\n';

		// to be subscribed to the controller for operation
		clamp_control_sub = nh.subscribe<std_msgs::Int16> ("controller", 1, &ClampControl::clamp_Callback, this);

		limit_switch_up_sub = nh.subscribe<std_msgs::Bool> ("switch_status_up", 1, &ClampControl::limit_up_Callback, this);
		limit_switch_down_sub = nh.subscribe<std_msgs::Bool> ("switch_status_down", 1, &ClampControl::limit_down_Callback, this);
		limit_switch_open_sub = nh.subscribe<std_msgs::Bool> ("switch_status_open", 1, &ClampControl::limit_open_Callback, this);
		limit_switch_close_sub = nh.subscribe<std_msgs::Bool> ("switch_status_close", 1, &ClampControl::limit_close_Callback, this);
		force_sub = nh.subscribe<std_msgs::Int16> ("force", 1, &ClampControl::force_Callback, this);
		stretch_sub = nh.subscribe<std_msgs::Float32> ("stretch_length", 1, &ClampControl::stretch_Callback, this);

        clamp_movement_pub = nh_.advertise<std_msgs::Float32>("clamp_movement", 1);
        clamp_grasp_pub = nh_.advertise<std_msgs::Float32>("clamp_grasp", 1);

		clamp_plate_status_pub = nh_.advertise<std_msgs::Bool>("clamp_plate_status", 1);
		grasp_status_pub = nh_.advertise<std_msgs::Bool>("grasp_status", 1, true);
        grasp_finished_pub = nh_.advertise<std_msgs::Bool>("grasp_finished", 1, true); // latched, meaning it waits for subscribers to send a message

        //signal(SIGINT, ClampControl::shutdownHandler);

        // Initialize States
		operation_mode = 0; // starting with raising clamp
        force = 0;
        stretch = 0;
	}


	void controller();
	void open_clamp();
	void lower_clamp();
	void raise_clamp();
	void close_clamp();
	void stretch_check();
	void check_grasp();

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
        if (stretch_window.size() >= window_size) {
            stretch_window.erase(stretch_window.begin());
            stretch_window.push_back(msg->data);
        }
        else {
            stretch_window.push_back(msg->data);
        }
        float sum = 0;
        for (int i = 0; i < stretch_window.size(); ++i) {
            sum += stretch_window.at(i);
        }
        stretch = sum/stretch_window.size();
	}

	void clamp_Callback(const std_msgs::Int16::ConstPtr& msg)
	{
		controller_value = msg -> data;
	}

    void shutdownHandler(int sig)
    {
        // Stop motion before shutting down
        std_msgs::Float32 clamp_movement_msg;
    	std_msgs::Float32 clamp_grasp_msg;
        clamp_movement_msg.data = 0.0;
        clamp_grasp_msg.data = 0.0;
        clamp_movement_pub.publish(clamp_movement_msg);
    	clamp_grasp_pub.publish(clamp_grasp_msg);

        ros::shutdown();
    }
};


void ClampControl::lower_clamp()
{
	if (limit_down == false)
	{
		clamp_movement = 1*clamp_scale;
	}
	else
	{
		clamp_movement = 0.0;
        operation_mode = 1;
	}
}

void ClampControl::open_clamp()
{
	if (limit_open == false)
	{
		clamp_grasp = 1*clamp_scale;
	}
	else
	{
		clamp_grasp = 0.0;
        operation_mode = 2;
	}
}

void ClampControl::stretch_check()
{
	if (stretch > 18.0)
	{
		clamp_plate_status = true;
	}
	else
	{
		clamp_plate_status = false;
	}
}

void ClampControl::close_clamp()
{
	if (limit_close == false)
	{
		clamp_grasp = -1*clamp_scale;
	}
	else
	{
		clamp_grasp = 0.0;
	}
}

void ClampControl::check_grasp()
{
	if ((clamp_plate_status == true) && (force > 900))
	{
		grasp_status = true;
        std_msgs::Bool grasp_status_msg;
        grasp_status_msg.data = grasp_status;
		operation_mode = 4;
	}
    else if ((clamp_plate_status == true) && (limit_close == true))
    {
        grasp_status = true;
        std_msgs::Bool grasp_status_msg;
        grasp_status_msg.data = grasp_status;
        operation_mode = 4;
    }
	else if (clamp_plate_status == false)
	{
        std::cout << "Clamp broken: " << clamp_plate_status << "\n";
		grasp_status = false;
        std_msgs::Bool grasp_status_msg;
        grasp_status_msg.data = grasp_status;
		operation_mode = 1;
	}
}

void ClampControl::raise_clamp()
{
	if (limit_up == false)
	{
		clamp_movement = -1*clamp_scale;
	}
	else
	{
		clamp_movement = 0.0;
        operation_mode = 5;
	}
}


void ClampControl::controller()
{
    // DEBUG:
    std::cout << "Operating mode: " << operation_mode << "\n";

    /* Operating Modes
     * 0 = Lowering the clamp
     * 1 = Opneing the clamp
     * 2 = Begin approach and wait for stretch sensor
     * 3 = Close the clamp and check force sensor for grasp
     * 4 = Raise the clamp
     * 5 = Indicate whether the grasp was successful
     */

	// Picking operation
	if (controller_value == 7)
	{
		if (operation_mode == 0)
		{
            std::cout << "Lowering\n";
			lower_clamp();
		}
		if (operation_mode == 1)
		{
            std::cout << "Opening\n";
			open_clamp();
		}
		if (operation_mode == 2)
		{
            std::cout << "Stretch checking\n";
			stretch_check();
            std::cout << "Stretch: " << clamp_plate_status << "\n";
            if (clamp_plate_status)
            {
                operation_mode = 3;
            }
		}
		if (operation_mode == 3)
		{
            std::cout << "Closing and grasping\n";
            stretch_check();
            std::cout << "Stretch: " << clamp_plate_status << "\n";
			close_clamp();
			check_grasp();
		}
		if (operation_mode == 4)
		{
            std::cout << "Raising\n";
			raise_clamp();
		}
		if (operation_mode == 5)
		{
            std::cout << "Grasp successful!\n";
			grasp_status == true;
            std_msgs::Bool grasp_finished_msg;
            grasp_finished_msg.data = true;
            grasp_finished_pub.publish(grasp_finished_msg);

            // FIXME: add this in when you create a service from the control handler that receives the success message and then changes the controller type before letting the operation mode turn to 0
            //operation_mode = 0;
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


	clamp_movement_msg.data = clamp_movement;
	clamp_grasp_msg.data = clamp_grasp;
	clamp_plate_status_msg.data = clamp_plate_status;


	clamp_movement_pub.publish(clamp_movement_msg);
	clamp_grasp_pub.publish(clamp_grasp_msg);
	clamp_plate_status_pub.publish(clamp_plate_status_msg);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "clamp_control");
    ros::init(argc, argv, "clamp_control", ros::init_options::NoSigintHandler);

	ClampControl clamp_control;

	ros::Rate rate(30);
	while (ros::ok())
	{
		clamp_control.controller();
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
