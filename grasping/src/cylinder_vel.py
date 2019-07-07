#!/usr/bin/env python


import rospy
import math
import time
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist


class GoForward():
    def __init__(self):
        rospy.init_node('cylinder_vel')
        rospy.Subscriber("/cylinder_detection/point", PointStamped, self.callback)
        rospy.Subscriber("/joy", Joy, self.joy_callback)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        r = rospy.Rate(10)

        # Cylinder position
        self.cylinder_x = 0
        self.cylinder_z = 0

        # Time values for when to search
        self.time_delay = 3
        self.cylinder_time = time.time()

        # Joystick control parameters
        self.joy_control = False

        while not rospy.is_shutdown():
            if not self.joy_control:
                self.system_time = time.time()
                print(self.system_time)
                time_diff = self.system_time - self.cylinder_time
                move_cmd = Twist()
                if time_diff > self.time_delay:
                    move_cmd.angular.z = -0.7
                else:
                    move_cmd.angular.z = 2.0*-math.atan2(self.cylinder_x,self.cylinder_z)

                self.previous_angular = move_cmd.angular.z
                self.cmd_vel.publish(move_cmd)
            r.sleep()

    def callback(self, msg):
        self.cylinder_x = -msg.point.y
        self.cylinder_z = msg.point.x
        self.cylinder_time = time.time()
        print(self.cylinder_time)

    def joy_callback(self, msg):
        if (msg.buttons[4] == 1):
            self.joy_control = True
        else:
            self.joy_control = False

    def shutdown(self):
        rospy.loginfo("Stop TurtleBot")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
   GoForward()
