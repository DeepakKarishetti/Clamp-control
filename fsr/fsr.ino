/* Pin layout
 *  Arduino ---> FSR
 *  FSR_R ---> 5V
 *  FSR_L ---> A0, R-qGND
 */

#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

std_msgs::Int16 force_msg;
ros::Publisher force_pub("force", &force_msg);

int fsrPin = 0;
int fsrReading;

void setup(void) {
  Serial.begin(9600);
  nh.initNode();
  nh.advertise(force_pub);
}

void loop(void) 
{
  fsrReading = analogRead(fsrPin);
  force_msg.data = fsrReading;
  force_pub.publish( &force_msg );
  // Serial.println(fsrReading);
  nh.spinOnce();
  delay(10);
}
