#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float32

num1 = 0.0
dest = 0.0

def callback(data):
    global dest
    dest = round(data.data, 2)

def smoother():
    global num1
    global dest

    rospy.init_node("smoother", anonymous=True)
    rospy.Subscriber("command", Float32, callback)
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        num1 = round(num1, 2)
        dest = round(dest, 2)
        if(num1 != dest):
            if(num1 < dest):
                num1 = math.fabs(num1 + 0.02)
            elif(num1 > dest):
                num1 = math.fabs(num1 - 0.02)
	print num1
	rate.sleep()
	
if __name__ == '__main__':
    smoother()
