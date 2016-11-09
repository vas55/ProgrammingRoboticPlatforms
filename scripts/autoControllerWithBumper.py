#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Empty
from kobuki_msgs.msg import BumperEvent

pub = rospy.Publisher("kobuki_command", Twist, queue_size=10)
odomReset = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
command = Twist()

positionX = 0.0
positionY = 0.0
degree = 0.0
tempDeg = 0.0
velocity = 0.0
distOffset = 0
bumperSet = 0
bumperHitStr = ""
cancelCommand = 0

def odomCallback(data):
    global positionX, positionY, degree
    # Convert quaternion to degree
    q = [data.pose.pose.orientation.x,
         data.pose.pose.orientation.y,
         data.pose.pose.orientation.z,
         data.pose.pose.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    # roll, pitch, and yaw are in radian
    degree = yaw * 180 / math.pi
    positionX = data.pose.pose.position.x
    positionY = data.pose.pose.position.y
    
def cleanUp():
    global pub, command
    command.linear.x = 0.0
    command.angular.z = 0.0
    pub.publish(command)
    rospy.sleep(1) 

def bumperCallback(data):
    global bumperSet, bumperHitStr
    if data.state != 0:
	bumperSet = 1

	if data.bumper == 0:
	    bumperHitStr = "***Left bumper is pressed***"
	elif data.bumper == 1:
	    bumperHitStr = "***Front bumper is pressed***"
	else:
	    bumperHitStr = "***Right bumper is pressed***"
    elif data.state == 0:
        bumperSet = 0

def runCommand(direction, maxSpeed, distance):
    global pub, command, positionX, positionY, degree, tempDeg, velocity, distOffset, bumperSet, bumperHitStr, cancelCommand
    if direction == "R" or direction == "L":
        tempDeg = abs(degree)
        bigTurn = 0

        if distance >= 360:
            distance = 359

	while tempDeg < distance and bumperSet != 1:           
	    tempDeg = abs(degree)
	    tempSpeed = abs(command.angular.z)
          
	    if distance > 179 and tempDeg > 10:
	        bigTurn = 1

	    if direction == "L" and degree < 0 and bigTurn == 1:
                tempDeg = 360 + degree

            if direction == "R" and degree > 0 and bigTurn == 1:
                tempDeg = 360 - degree
 
	    if tempDeg < distance/3:
		if tempSpeed < maxSpeed:
		    tempSpeed = tempSpeed + 0.002
		    distOffset = tempDeg
	    elif tempDeg >= (distance - distOffset):
		if tempSpeed > 0.02:
		    tempSpeed = tempSpeed - 0.002 
				
	    if direction == "R":
	        tempSpeed = tempSpeed * -1

	    command.angular.z = tempSpeed
				
	    pub.publish(command)
	    rospy.sleep(0.01)
				

    elif direction == "F" or direction == "B":
        while abs(positionX) < distance and bumperSet != 1:
	    velocity = 0.7*command.linear.x
	    tempX = abs(positionX)
	    tempSpeed = abs(command.linear.x)

	    if tempX < distance/3:
		if tempSpeed < maxSpeed:
	            tempSpeed = tempSpeed + 0.002
		    distOffset = tempX
	    elif tempX >= (distance - distOffset):
		if tempSpeed > 0.02:
		    tempSpeed = tempSpeed - 0.004

	    if(direction == "B"):
		tempSpeed = tempSpeed*-1
		
	    command.linear.x = tempSpeed

	    pub.publish(command)
	    rospy.sleep(0.01)

    if bumperSet == 1:
	bumperSet = 0
        cancelCommand = 1
	print bumperHitStr	

    command.linear.x = 0.0
    command.angular.z = 0.0

    pub.publish(command)

    
def autoController():
    global pub, command, positionX, positionY, degree, velocity, cancelCommand
    rospy.init_node("autoControl", anonymous=True)
    rospy.Subscriber('/odom', Odometry, odomCallback)
    rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bumperCallback)
    rospy.on_shutdown(cleanUp)

    while pub.get_num_connections() == 0:
        pass

    while not rospy.is_shutdown():
        odomReset.publish(Empty())
        mode = input("Select Mode (1 for single, 2 for multiple): ")

        if mode == 1:
            inputCommand = raw_input("Enter a command and press Enter to execute: ")
            inputCommand = inputCommand.split(" ")
            direction = inputCommand[0]
            maxSpeed = float(inputCommand[1])
            distance = float(inputCommand[2])

            if(maxSpeed > 0.8):
                maxSpeed = 0.8
	    
	    runCommand(direction, maxSpeed, distance)
	   
        
	elif mode == 2:
            commands = raw_input("Enter a series of commands and press Enter to execute: ")
            commands = commands.split(",")

            for com in commands:
                odomReset.publish(Empty())
                rospy.sleep(0.1)

                com = com.strip()
                com = com.split(" ")
                direction = com[0]
                maxSpeed = float(com[1])
                distance = float(com[2])

                if(maxSpeed > 0.8):
                    maxSpeed = 0.8
		
                if cancelCommand != 1:	
	            runCommand(direction, maxSpeed, distance)
            
            cancelCommand = 0
        rospy.sleep(0.1)

if __name__ == '__main__':
    autoController()
