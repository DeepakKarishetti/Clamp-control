/* Pin layout
 *  Arduino ---> Clamp control
 *  Red ---> GND  
 *  White ---> 5V  
 *  Black ---> Signal 1 
 *  Green ---> Signal 2 
 *  
 *  Limit switch ---> Arduino
 *  Clamp_movement & Clamp_grasp
 *  COM ---> 5V
 *  NC ---> R-7, GNDx
 *  
 *  FSR ---> Arduino 
 *  FSR_R ---> 5V
 *  FSR_L ---> R-GND, A0 
 *  
 *  Stretch sensor ---> Arduino
 *  SS_r ---> 5V 
 *  SS_l ---> 10k R -- GND, A1
 */

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>

std_msgs::Int16 force_msg;
std_msgs::Bool switch_up_msg;
std_msgs::Bool switch_down_msg;
std_msgs::Bool switch_open_msg;
std_msgs::Float32 stretch_msg;

void switchCallback(const std_msgs::Bool&);
void clampmovementCallback(const std_msgs::Float32&);
void clampgraspCallback(const std_msgs::Float32&);

// Limit switch
int limit_switch_up = 7;
int limit_switch_down = 6;
int limit_switch_open = 5;
const int led = 13;
bool switch_status_up;
bool switch_status_down;
bool switch_status_open;

// Force Sensitive Resistor
int fsrPin = 0;
int fsrReading;

// Stretch sensor
int stretch_sensor_1_pin = 1;
int stretch_sensor_2_pin = 2;

// Clamp switch
const int SIGNAL_PIN_1 = 12;
const int SIGNAL_PIN_2 = 11;

const int SIGNAL_PIN_3 = 10;
const int SIGNAL_PIN_4 = 9;

const int PWM_MIN_1 = 120;
const int PWM_MIDDLE_1 = 190;
const int PWM_MAX_1 = 255;

const int PWM_MIN_2 = 80;
const int PWM_MIDDLE_2 = 127;
const int PWM_MAX_2 = 170;

bool clamp_switch;
float clamp_movement;
float clamp_grasp;

// ROS
ros::NodeHandle nh;

// Limit switch
ros::Publisher limit_switch_up_pub("switch_status_up", &switch_up_msg);
ros::Publisher limit_switch_down_pub("switch_status_down", &switch_down_msg);
ros::Publisher limit_switch_open_pub("switch_status_open", &switch_open_msg);

// FSR
ros::Publisher force_pub("force", &force_msg);

// Stretch sensor
ros::Publisher stretch_sensor_pub("stretch_length", &stretch_msg);

// Clamp switch 
ros::Subscriber<std_msgs::Bool> clamp_switch_sub("clamp_switch_node/clamp_switch", &switchCallback);
ros::Subscriber<std_msgs::Float32> clamp_movement_sub("clamp_switch_node/clamp_movement", &clampmovementCallback);
ros::Subscriber<std_msgs::Float32> clamp_grasp_sub("clamp_switch_node/clamp_grasp", &clampgraspCallback);
//
void setup() 
{
  Serial.begin(57600);
  nh.initNode();
  
  // Limit switch
  pinMode(limit_switch_up, INPUT);
  pinMode(led, OUTPUT);
  digitalWrite(led,HIGH);
  nh.advertise(limit_switch_up_pub);

  pinMode(limit_switch_down, INPUT);
  pinMode(led, OUTPUT);
  digitalWrite(led,HIGH);
  nh.advertise(limit_switch_down_pub);

  pinMode(limit_switch_open, INPUT);
  pinMode(led, OUTPUT);
  digitalWrite(led,HIGH);
  nh.advertise(limit_switch_open_pub);

  // FSR
  nh.advertise(force_pub);

  // Stretch sensor
  nh.advertise(stretch_sensor_pub);
  
  // Clamp
  nh.subscribe(clamp_switch_sub);
  nh.subscribe(clamp_movement_sub);
  nh.subscribe(clamp_grasp_sub);

  pinMode(SIGNAL_PIN_1, OUTPUT);
  pinMode(SIGNAL_PIN_2, OUTPUT);
  pinMode(SIGNAL_PIN_3, OUTPUT);
  pinMode(SIGNAL_PIN_4, OUTPUT);
  
  analogWrite(SIGNAL_PIN_1, PWM_MIDDLE_1);
  analogWrite(SIGNAL_PIN_2, PWM_MIDDLE_2);
  analogWrite(SIGNAL_PIN_3, PWM_MIDDLE_1);
  analogWrite(SIGNAL_PIN_4, PWM_MIDDLE_2);
}

void loop() 
{
  
  // Limit switch
  // Up
  if (digitalRead(limit_switch_up) == LOW)
  {
    digitalWrite(led, HIGH);
    switch_status_up = true;
  }
  else
  {
    digitalWrite(led, LOW);
    switch_status_up = false;
  }
  switch_up_msg.data = switch_status_up;
  limit_switch_up_pub.publish(&switch_up_msg);

  // Down
  if (digitalRead(limit_switch_down) == LOW)
  {
    digitalWrite(led, HIGH);
    switch_status_down = true;
  }
  else
  {
    digitalWrite(led, LOW);
    switch_status_down = false;
  }
  switch_down_msg.data = switch_status_down;
  limit_switch_down_pub.publish(&switch_down_msg);

  // Clamp open limit switch
  if (digitalRead(limit_switch_open) == LOW)
  {
    digitalWrite(led, HIGH);
    switch_status_open = true;
  }
  else
  {
    digitalWrite(led, LOW);
    switch_status_open = false;
  }
  switch_open_msg.data = switch_status_open;
  limit_switch_open_pub.publish(&switch_open_msg);
  
  // FSR
  fsrReading = analogRead(fsrPin);
  force_msg.data = fsrReading;
  force_pub.publish(&force_msg);

  // Stretch sensor
  float stretch_value;
  
  int value_1;
  int v_1_in = 5;
  float v_1_out = 0;
  float r_1_1 = 10;
  float r_1_2 = 0;
  float val_1;
  float buffer_1 = 0;
  value_1 = analogRead(stretch_sensor_1_pin);
  v_1_out = (5.0 / 1023.0) * value_1;
  buffer_1 = (v_1_in / v_1_out) - 1;
  r_1_2 = r_1_1 / buffer_1;
  val_1 = 1000 / r_1_2;
  
  int value_2;
  int v_2_in = 5;
  float v_2_out = 0;
  float r_2_1 = 10;
  float r_2_2 = 0;
  float val_2;
  float buffer_2 = 0;
  value_2 = analogRead(stretch_sensor_2_pin);
  v_2_out = (5.0 / 1023.0) * value_2;
  buffer_2 = (v_2_in / v_2_out) - 1;
  r_2_2 = r_2_1 / buffer_2;
  val_2 = 1000 / r_2_2;

  stretch_value = (val_1 + val_2) / 2;
  
  stretch_msg.data = stretch_value;
  stretch_sensor_pub.publish( &stretch_msg );
  
  // Clamp switch control

  // Clamp movement up and down
  if (clamp_movement <= 0)
  {
  // Check for the limit switch
    if (switch_status_up == false)
    {
      int pwm_signal_move_1 = map(100*clamp_movement, -100, 100, PWM_MIN_1, PWM_MAX_1);
      int pwm_signal_move_2 = map(100*clamp_movement, -100, 100, PWM_MAX_2, PWM_MIN_2);

      analogWrite(SIGNAL_PIN_1, pwm_signal_move_1);
      analogWrite(SIGNAL_PIN_2, pwm_signal_move_2);
    }
    else
    {
      analogWrite(SIGNAL_PIN_1, PWM_MIDDLE_1);
      analogWrite(SIGNAL_PIN_2, PWM_MIDDLE_2);
    }
  }
  
  else if (clamp_movement > 0)
  {
    if (switch_status_down == false)
    {
      int pwm_signal_move_1 = map(100*clamp_movement, -100, 100, PWM_MIN_1, PWM_MAX_1);
      int pwm_signal_move_2 = map(100*clamp_movement, -100, 100, PWM_MAX_2, PWM_MIN_2);

      analogWrite(SIGNAL_PIN_1, pwm_signal_move_1);
      analogWrite(SIGNAL_PIN_2, pwm_signal_move_2);
    }
    else
    {
      analogWrite(SIGNAL_PIN_1, PWM_MIDDLE_1);
      analogWrite(SIGNAL_PIN_2, PWM_MIDDLE_2);
    }
  }
  
  // Clamp open and close
  
  if (clamp_grasp <= 0) 
  {
    // Checking for the plate position
    //if ((stretch_value > 16.0 && stretch_value < 22.0) && (fsrReading < 400))
    if (true)
    {
      // Check for FSR value and limit switch
      //if (fsrReading < 400)
      int pwm_signal_grasp_1 = map(100*clamp_grasp, -100, 100, PWM_MIN_1, PWM_MAX_1);
      int pwm_signal_grasp_2 = map(100*clamp_grasp, -100, 100, PWM_MAX_2, PWM_MIN_2);
  
      analogWrite(SIGNAL_PIN_3, pwm_signal_grasp_1);
      analogWrite(SIGNAL_PIN_4, pwm_signal_grasp_2);
       
    }
    else
    {
      analogWrite(SIGNAL_PIN_3, PWM_MIDDLE_1);
      analogWrite(SIGNAL_PIN_4, PWM_MIDDLE_2);
    }
  }
  else if (clamp_grasp > 0)
  {
    // Check for limit switch
    if (switch_status_open == false)
    {
      int pwm_signal_grasp_1 = map(100*clamp_grasp, -100, 100, PWM_MIN_1, PWM_MAX_1);
      int pwm_signal_grasp_2 = map(100*clamp_grasp, -100, 100, PWM_MAX_2, PWM_MIN_2);
  
      analogWrite(SIGNAL_PIN_3, pwm_signal_grasp_1);
      analogWrite(SIGNAL_PIN_4, pwm_signal_grasp_2);
    } 
    else
    {
      analogWrite(SIGNAL_PIN_3, PWM_MIDDLE_1);
      analogWrite(SIGNAL_PIN_4, PWM_MIDDLE_2);
    }
  }
  
  nh.spinOnce();
  delay(1);
}

// Clamp switch
void switchCallback(const std_msgs::Bool& msg)
{
  clamp_switch = msg.data;
}

// Clamp movement
void clampmovementCallback(const std_msgs::Float32& msg)
{
  clamp_movement = msg.data;
}

// Clamp grasp
void clampgraspCallback(const std_msgs::Float32& msg)
{
  clamp_grasp = msg.data;
}
