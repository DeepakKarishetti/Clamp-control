/* Pin layout
 * 
 *  Stretch sensor ---> Arduino
 *  SS_r ---> 5V 
 *  SS_l ---> 10k R -- GND, A1
 *
 */ 
#include <ros.h>
#include <std_msgs/Float32.h>

std_msgs::Float32 stretch_msg;

//ROS
ros::Publisher stretch_sensor_pub("stretch_length", &stretch_msg);

// Stretch sensor
int stretch_sensor_1_pin = 1;
int stretch_sensor_2_pin = 2;

ros::NodeHandle nh;

void setup() 
{
  Serial.begin(115200);
  nh.advertise(stretch_sensor_pub);

}
  
void loop() 
{
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
  stretch_value = (val_1 + val_2) / 2;
  
  stretch_msg.data = stretch_value;
  stretch_sensor_pub.publish( &stretch_msg );
}   
