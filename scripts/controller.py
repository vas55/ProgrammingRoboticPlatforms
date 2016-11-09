#!/usr/bin/env python 

import rospy
from std_msgs.msg import Float32

def controller():
    pub = rospy.Publisher("command", Float32, queue_size = 10)
    rospy.init_node("controller", anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        usr_string = input("Please enter a floating-point number: ")
	print usr_string
	pub.publish(usr_string)
	rate.sleep()

if __name__ == '__main__':
    try:
	controller()
    except rospy.ROSInterruptException:
	pass
