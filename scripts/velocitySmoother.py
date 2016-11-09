#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist


pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
currentCommand = Twist()
currentCommand.linear.x = 0.0
currentCommand.angular.z = 0.0
targetCommand = Twist()
targetCommand.linear.x = 0.0
targetCommand.angular.z = 0.0

def updateCommand(data):
    global targetCommand
    targetCommand = data
    targetCommand.linear.x = round(targetCommand.linear.x, 2)
    targetCommand.angular.z = round(targetCommand.angular.z, 2)

def cleanUp():
    global currentCommand
    currentCommand.linear.x = 0.0
    currentCommand.angular.z = 0.0
    pub.publish(currentCommand)
    rospy.sleep(1)

def velSmoother():
    global pub, targetCommand, currentCommand
    rospy.init_node("velocitySmoother", anonymous=True)
    rospy.Subscriber("kobuki_command", Twist, updateCommand)
    rospy.on_shutdown(cleanUp)

    while pub.get_num_connections() == 0:
        pass

    while not rospy.is_shutdown():
        currentCommand.linear.x = round(currentCommand.linear.x, 2)
        currentCommand.angular.z = round(currentCommand.angular.z, 2)

        if currentCommand.linear.x < targetCommand.linear.x:
            currentCommand.linear.x = currentCommand.linear.x + 0.02
        elif currentCommand.linear.x > targetCommand.linear.x:
            currentCommand.linear.x = currentCommand.linear.x - 0.02

        if currentCommand.angular.z < targetCommand.angular.z:
            currentCommand.angular.z = currentCommand.angular.z + 0.04
        elif currentCommand.angular.z > targetCommand.angular.z:
            currentCommand.angular.z = currentCommand.angular.z - 0.04

        if targetCommand.linear.x == -2.0:
            currentCommand.linear.x = 0.0
            targetCommand.linear.x = 0.0
        if targetCommand.angular.z == -2.0:
            currentCommand.angular.z = 0.0
            targetCommand.angular.z = 0.0

        pub.publish(currentCommand)
        rospy.sleep(0.1)


if __name__ == '__main__':
    velSmoother()

