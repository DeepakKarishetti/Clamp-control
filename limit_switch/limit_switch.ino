/* Pin layout
 *  Arduino ---> Limit switch
 *  COM ---> 5V,  7
 *  NC ---> GND
 */

#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;

std_msgs::Bool switch_msg;
ros::Publisher switch_pub("limit_switch", &switch_msg);

int limit_switch = 7;
int led = 13;
bool switch_status;

void setup() {
  Serial.begin(9600);
  nh.initNode();
  nh.advertise(switch_pub);
  
  pinMode(limit_switch, INPUT);
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
}

void loop() {
  if(digitalRead(limit_switch) == LOW) {
    digitalWrite(led, HIGH);
    switch_status = true;
    Serial.println(switch_status);
    switch_msg.data = switch_status;
    switch_pub.publish( &switch_msg );
  }
  else {
    digitalWrite(led, LOW);
    switch_status = false;
    Serial.println(switch_status);
    switch_msg.data = switch_status;
    switch_pub.publish( &switch_msg );
  }
  nh.spinOnce();
  delay(1);
}
