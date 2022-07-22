#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

twist = Twist()

cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)

if __name__ == "__main__":
    rospy.init_node('stop', anonymous=True)

    twist.linear.x = 0.0
    twist.angular.z = 0.0
    while True:
        cmd_vel_pub.publish(twist)