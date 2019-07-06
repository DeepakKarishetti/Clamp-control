/* Pin layout
 *  Arduino ---> Clamp control
 *  Red ---> GND  
 *  White ---> 5V  
 *  Black ---> Signal 1 
 *  Green ---> Signal 2 
 */

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

void switchCallback(const std_msgs::Bool&);
void clampmovementCallback(const std_msgs::Float32&);
void clampgraspCallback(const std_msgs::Float32&);

const int SIGNAL_PIN_1 = 3;
const int SIGNAL_PIN_2 = 4;
const int DEBUG_LED = 13;

const int PWM_MIN = 0;
const int PWM_MIDDLE = 127;
const int PWM_MAX = 255;
bool clamp_switch;
float clamp_movement;
float clamp_grasp;


// ROS 
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Bool> clamp_switch_sub("clamp_switch_node/clamp_switch", &switchCallback);
ros::Subscriber<std_msgs::Float32> clamp_movement_sub("clamp_switch_node/clamp_movement", &clampmovementCallback);
ros::Subscriber<std_msgs::Float32> clamp_grasp_sub("clamp_switch_node/clamp_grasp", &clampgraspCallback);

void setup() 
{
  nh.initNode();
  nh.subscribe(clamp_switch_sub);
  nh.subscribe(clamp_movement_sub);
  nh.subscribe(clamp_grasp_sub);

  pinMode(SIGNAL_PIN_1, OUTPUT);
  pinMode(SIGNAL_PIN_2, OUTPUT);
  pinMode(DEBUG_LED, OUTPUT);
  
  analogWrite(SIGNAL_PIN_1, PWM_MIDDLE);
  analogWrite(SIGNAL_PIN_2, PWM_MIDDLE);
  // clamp_movement = 0.75;
}

void loop() 
{
  // Clamp switch control
  if (clamp_movement != 0)
  {
    int pwm_signal_move_1 = map(100*clamp_movement, -100, 100, PWM_MIN, PWM_MAX);
    int pwm_signal_move_2 = map(100*clamp_movement, -100, 100, PWM_MAX, PWM_MIN);

    analogWrite(SIGNAL_PIN_1, pwm_signal_move_1);
    analogWrite(SIGNAL_PIN_2, pwm_signal_move_2);
  }
  else
  {
    int pwm_signal_grasp_1 = map(100*clamp_grasp, -100, 100, PWM_MIN, PWM_MAX);
    int pwm_signal_grasp_2 = map(100*clamp_grasp, -100, 100, PWM_MAX, PWM_MIN);
  
    analogWrite(SIGNAL_PIN_1, pwm_signal_grasp_1);
    analogWrite(SIGNAL_PIN_2, pwm_signal_grasp_2);
  }
  
  nh.spinOnce();
  delay(1);
}

void switchCallback(const std_msgs::Bool& msg)
{
  clamp_switch = msg.data;
}

void clampmovementCallback(const std_msgs::Float32& msg)
{
  clamp_movement = msg.data;
}

void clampgraspCallback(const std_msgs::Float32& msg)
{
  clamp_grasp = msg.data;
}
