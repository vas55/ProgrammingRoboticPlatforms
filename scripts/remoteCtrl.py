#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

pub = rospy.Publisher("kobuki_command", Twist, queue_size=10)
command = Twist()

def joystickCallback(data):
    global pub, command
    
    x = 0.4 + (-0.4)*data.axes[5]

    if data.buttons[0] == 1:
        x = (-1)*x

    if x > 0.8:
        x = 0.8
    elif x < -0.8:
        x = -0.8

    z = data.axes[0]

    if z > 0.8:
        z = 0.8
    elif z < -0.8:
        z = -0.8

    if data.axes[2] <= 0:
        x = -2.0
        z = -2.0

    if data.buttons[1] == 1:
        x = -2.0
        y = -2.0
        rospy.signal_shutdown("Emergency Stop!!!")

    command.linear.x = x
    command.angular.z = z
    pub.publish(command)

def cleanUp():
    global pub, command
    command.linear.x = -2.0
    command.angular.z = -2.0
    pub.publish(command)
    rospy.sleep(1)

def remoteController():
    rospy.init_node("remoteControl", anonymous=True)
    rospy.Subscriber("joy", Joy, joystickCallback)
    rospy.on_shutdown(cleanUp)

    #while pub.get_num_connections() == 0:
    #    pass

    rospy.spin()

if __name__ == '__main__':
    remoteController()
