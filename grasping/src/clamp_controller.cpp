#include <iostream>
#include <vector>
#include <algorithm> // std::find
#include <sys/time.h>
#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Joy.h>

class ClampControl
{
private:
	ros::NodeHandle nh;
    ros::NodeHandle nh_;

    ros::Subscriber control_mode_sub;

	ros::Subscriber limit_switch_up_sub;
	ros::Subscriber limit_switch_down_sub;
	ros::Subscriber limit_switch_open_sub;
	ros::Subscriber limit_switch_close_sub;
    ros::Subscriber limit_switch_plate_sub;
	ros::Subscriber force_sub;
	ros::Subscriber stretch_sub;
    ros::Subscriber joystick_override_sub;

    ros::Publisher clamp_movement_pub;
    ros::Publisher clamp_grasp_pub;
    static ros::Publisher clamp_movement_pub_global;
    static ros::Publisher clamp_grasp_pub_global;

	ros::Publisher clamp_plate_status_pub;
	ros::Publisher grasp_status_pub;
    ros::Publisher grasp_finished_pub;

	bool limit_up;
	bool limit_down;
	bool limit_open;
	bool limit_close;
    bool limit_plate;
	int force;
    int force_threshold;
	float stretch;
    float stretch_threshold;
	int control_mode;
    std::vector<int> available_control_modes; // vector of possible values for turing on this controller

    double clamp_scale_movement; // adjusts value sent to the arduino controlling the clamp, range: [0, 1]
	double clamp_scale_grasp;
    float clamp_grasp;
	float clamp_movement;
	bool clamp_plate_status;
	bool grasp_status;
	int operation_mode;

    int window_size = 5;
    std::vector<float> stretch_window;

    // Joystick Controller Variables
    double timeout, timeout_start;
    int autonomous_deadman_button, manual_deadman_button;
    bool autonomous_deadman_on, manual_deadman_on; // deadman switch

public:
	ClampControl() : nh(""), nh_("~")
	{
        // Set parameters
        nh_.param<double>("clamp_scale_movement", clamp_scale_movement, 0.5);
        nh_.param<double>("clamp_scale_grasp", clamp_scale_grasp, 0.5);
        nh_.param("manual_deadman", manual_deadman_button, 4);
        nh_.param("autonomous_deadman", autonomous_deadman_button, 5);
        nh_.param("timeout", timeout, 1.0);

        std::cout << "Clamp scale movement: " << clamp_scale_movement << '\n';
        std::cout << "Clamp scale grasp: " << clamp_scale_grasp << '\n';

        clamp_movement_pub = nh_.advertise<std_msgs::Float32>("clamp_movement", 1);
        clamp_grasp_pub = nh_.advertise<std_msgs::Float32>("clamp_grasp", 1);
		clamp_plate_status_pub = nh_.advertise<std_msgs::Bool>("clamp_plate_status", 1);
		grasp_status_pub = nh_.advertise<std_msgs::Bool>("grasp_status", 1, true);
        grasp_finished_pub = nh_.advertise<std_msgs::Bool>("grasp_finished", 1, true); // latched, meaning it waits for subscribers to send a message

		// to be subscribed to the controller for operation
		control_mode_sub = nh.subscribe<std_msgs::Int8> ("/control_mode", 1, &ClampControl::controlModeCallback, this);

		limit_switch_up_sub = nh.subscribe<std_msgs::Bool> ("switch_status_up", 1, &ClampControl::limit_up_Callback, this);
		limit_switch_down_sub = nh.subscribe<std_msgs::Bool> ("switch_status_down", 1, &ClampControl::limit_down_Callback, this);
		limit_switch_open_sub = nh.subscribe<std_msgs::Bool> ("switch_status_open", 1, &ClampControl::limit_open_Callback, this);
		limit_switch_close_sub = nh.subscribe<std_msgs::Bool> ("switch_status_close", 1, &ClampControl::limit_close_Callback, this);
        limit_switch_plate_sub = nh.subscribe<std_msgs::Bool> ("switch_status_plate", 1, &ClampControl::limit_plate_Callback, this);
		force_sub = nh.subscribe<std_msgs::Int16> ("force", 1, &ClampControl::force_Callback, this);
		stretch_sub = nh.subscribe<std_msgs::Float32> ("stretch_length", 1, &ClampControl::stretch_Callback, this);
        joystick_override_sub = nh_.subscribe("/joy", 1, &ClampControl::joystickCallback, this);

        //signal(SIGINT, ClampControl::shutdownHandler);

        // Initialize States
		operation_mode = 0; // starting with raising clamp
        force = 0;
        force_threshold = 800;
        stretch = 0;
        stretch_threshold = 17.0;

        //===== Print out possible values for control mode =====//
        // Pushback more numbers to allow this controller to operate in more
        // modes
        available_control_modes.push_back(3);
        std::string message = "Available control_modes for [" + ros::this_node::getName() + "]: ";
        for (int i = 0; i < available_control_modes.size(); ++i) {
            char msg_buffer[10]; // increase size if more digits are needed
            sprintf(msg_buffer, "%d", available_control_modes.at(i));
            message += msg_buffer;
            if (i != available_control_modes.size() - 1) {
                message += ", ";
            }
            else {
                message += '\n';
            }
        }
        ROS_INFO("%s", message.c_str());

        // Initialize joystick variables
        timeout_start = getWallTime();
        autonomous_deadman_on = false;
        manual_deadman_on = false;
	}


	void controller();
	void open_clamp();
	void lower_clamp();
	void raise_clamp();
	void close_clamp();
	void plate_check();
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

    void limit_plate_Callback(const std_msgs::Bool::ConstPtr& msg)
    {
        limit_plate = msg -> data;
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

	void controlModeCallback(const std_msgs::Int8::ConstPtr& msg)
	{
		control_mode = msg -> data;
	}

    void joystickCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        // Update timeout time
        timeout_start = getWallTime();

        // Update deadman buttons
        if (msg->buttons[manual_deadman_button] == 1) {
            manual_deadman_on = true;
        }
		else {
            manual_deadman_on = false;
        }

        if (msg->buttons[autonomous_deadman_button] == 1) {
            autonomous_deadman_on = true;
        }
		else {
            autonomous_deadman_on = false;
        }
    }

    void publishStopCommand()
    {
        // Stop motion
        std_msgs::Float32 clamp_movement_msg;
    	std_msgs::Float32 clamp_grasp_msg;
        clamp_movement_msg.data = 0.0;
        clamp_grasp_msg.data = 0.0;

        // DEBUG:
        std::cout << "[clamp_control]: publishing clamp movement stop command\n";

        clamp_movement_pub.publish(clamp_movement_msg);
    	clamp_grasp_pub.publish(clamp_grasp_msg);
    }

    double getWallTime() {
        struct timeval time;
        if (gettimeofday(&time, NULL)) {
            return 0;
        }
        return (double)time.tv_sec + (double)time.tv_usec*0.000001;
    }
};


void ClampControl::lower_clamp()
{
	if (limit_down == false)
	{
		clamp_movement = 1*clamp_scale_movement;
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
		clamp_grasp = 1*clamp_scale_grasp;
	}
	else
	{
		clamp_grasp = 0.0;
        operation_mode = 2;
	}
}

void ClampControl::plate_check()
{
	if (stretch > stretch_threshold || limit_plate == true)
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
	if (limit_close == false && force < force_threshold)
	{
		clamp_grasp = -1*clamp_scale_grasp;
	}
	else
	{
		clamp_grasp = 0.0;
	}
}

void ClampControl::check_grasp()
{
	if ((clamp_plate_status == true) && (force >= force_threshold))
	{
		grasp_status = true;
        std_msgs::Bool grasp_status_msg;
        grasp_status_msg.data = grasp_status;
        grasp_status_pub.publish(grasp_status_msg);
		operation_mode = 4;
	}
    else if ((clamp_plate_status == true) && (limit_close == true))
    {
        grasp_status = true;
        std_msgs::Bool grasp_status_msg;
        grasp_status_msg.data = grasp_status;
        grasp_status_pub.publish(grasp_status_msg);
        operation_mode = 4;
    }
	else if ((clamp_plate_status == false) && (limit_close == true) && (force < force_threshold))
	{
        std::cout << "Clamp broken: " << clamp_plate_status << "\n";
		grasp_status = false;
        std_msgs::Bool grasp_status_msg;
        grasp_status_msg.data = grasp_status;
        grasp_status_pub.publish(grasp_status_msg);
		operation_mode = 1;
	}
}

void ClampControl::raise_clamp() {
	if (limit_up == false)
	{
		clamp_movement = -1*clamp_scale_movement;
	}
	else
	{
		clamp_movement = 0.0;
        operation_mode = 5;
	}
}

bool checkControlMode(int mode, std::vector<int> vector_of_modes)
{
    // Use 'find' on the vector to determine existence of 'mode'
    std::vector<int>::iterator it;
    it = std::find(vector_of_modes.begin(), vector_of_modes.end(), mode);
    if (it != vector_of_modes.end()) {
        return true;
    }
    else {
        return false;
    }
}

void ClampControl::controller()
{
    if (checkControlMode(control_mode, available_control_modes)) {
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
            std::cout << "Stretch/plate switch checking\n";
    		plate_check();
            std::cout << "plate: " << clamp_plate_status << "\n";
            if (clamp_plate_status)
            {
                operation_mode = 3;
            }
    	}
    	if (operation_mode == 3)
    	{
            std::cout << "Closing and grasping\n";
            plate_check();
            std::cout << "plate: " << clamp_plate_status << ", force: " << force << "\n";
    		check_grasp();
            close_clamp();
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

    	std_msgs::Float32 clamp_movement_msg;
    	std_msgs::Float32 clamp_grasp_msg;
    	std_msgs::Bool clamp_plate_status_msg;

        if (manual_deadman_on and ((getWallTime() - timeout_start) < timeout)) {
            // Send no command
        }
        else if (autonomous_deadman_on and ((getWallTime() - timeout_start) < timeout)) {
            // Send clamp control commands
            clamp_movement_msg.data = clamp_movement;
        	clamp_grasp_msg.data = clamp_grasp;
            clamp_movement_pub.publish(clamp_movement_msg);
        	clamp_grasp_pub.publish(clamp_grasp_msg);
        }
        else {
            // Joystick has timed out, send 0 velocity commands for clamp
            clamp_movement_msg.data = 0;
        	clamp_grasp_msg.data = 0;
            clamp_movement_pub.publish(clamp_movement_msg);
        	clamp_grasp_pub.publish(clamp_grasp_msg);
        }

        // Stretch sensor status can be sent everytime
        // update plate status
        plate_check();
        clamp_plate_status_msg.data = clamp_plate_status;
        clamp_plate_status_pub.publish(clamp_plate_status_msg);
    }
}

//============================================================================//
// Need Global pointer to use with ROS shutdown callback
//============================================================================//
ClampControl* clamp_control;

void myShutdown(int sig)
{
    // DEBUG:
    ROS_INFO("[%s]: Stopping clamp movement\n", ros::this_node::getName().c_str());

    // Stop motion before shutting down
    clamp_control->publishStopCommand();

    ros::shutdown();
}

int main(int argc, char **argv)
{
	//ros::init(argc, argv, "clamp_control");
    ros::init(argc, argv, "clamp_control", ros::init_options::NoSigintHandler);

	//ClampControl clamp_control;
    clamp_control = new ClampControl();

    signal(SIGINT, &myShutdown);

	ros::Rate rate(30);
	while (ros::ok())
	{
		//clamp_control.controller();
        clamp_control->controller();
		ros::spinOnce();
		rate.sleep();
	}

    delete clamp_control;

	return 0;
}
