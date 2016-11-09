#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import ButtonEvent

pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
currentCommand = Twist()
currentCommand.linear.x = 0.0
currentCommand.angular.z = 0.0

def updateCommand(data):
    global currentCommand
    currentCommand = data

def cleanUp():
    global currentCommand
    currentCommand.linear.x = 0.0
    currentCommand.angular.z = 0.0
    pub.publish(currentCommand)
    rospy.sleep(1)

def buttonCallback(data):
    global currentCommand, pub
    if data.state == ButtonEvent.PRESSED:
	currentCommand.linear.x = 0
	currentCommand.angular.z = 0
	rospy.sleep(1)		
        rospy.signal_shutdown("Emergency Stop!!!")
   
def constCommand():
    global pub, currentCommand
    rospy.init_node("constCommand", anonymous=True)
    rospy.Subscriber("kobuki_command", Twist, updateCommand)
    rospy.Subscriber("/mobile_base/events/button", ButtonEvent, buttonCallback)
    rospy.on_shutdown(cleanUp)

    while pub.get_num_connections() == 0:
        pass

    while not rospy.is_shutdown():
        pub.publish(currentCommand)
        rospy.sleep(0.1)


if __name__ == '__main__':
    constCommand()

