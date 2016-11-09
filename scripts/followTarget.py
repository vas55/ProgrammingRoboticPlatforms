#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from struct import *

depthData = Image();
isDepthReady = False;

def depthCallback(data):
    global depthData, isDepthReady
    depthData = data
    isDepthReady = True

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

def updateColorImage(data):
    global colorImage, isColorImageReady
    colorImage = data
    isColorImageReady = True

def runCommand(direction, maxSpeed, distance, angleNumber):
    global pub, command, positionX, positionY, degree, tempDeg, velocity, distOffset
    if direction == "R" or direction == "L":
        tempDeg = abs(degree)
        bigTurn = 0

        if distance >= 360:
            distance = 359

	while tempDeg < distance:
            findBallAndGoal(direction, angleNumber)         
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
		    tempSpeed = tempSpeed + 0.02
		    distOffset = tempDeg
	    elif tempDeg >= (distance - distOffset):
		if tempSpeed > 0.02:
		    tempSpeed = tempSpeed - 0.02 
				
	    if direction == "R":
	        tempSpeed = tempSpeed * -1

	    command.angular.z = tempSpeed
				
	    pub.publish(command)
	    rospy.sleep(0.01)
				

    elif direction == "F" or direction == "B":
        while abs(positionX) < distance:
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

    command.linear.x = 0.0
    command.angular.z = 0.0

    pub.publish(command)
    odomReset.publish(Empty())
    rospy.sleep(0.5)

def move(distance):

    #will go forward if greater than 1.5
    if (distance > 1.5):
	maxspeed = 0.8
	direction = "F"
	
	runCommand(direction, maxSpeed, distance)

    #will stop robot if less than 1.5 distance
    if direction = "F" and distance < 1.5:
	maxspeed = 0.0
	runCommand(direction, maxSpeed, distance)

    #using PID to go faster or slower. Like it worked last time...
    
def main():
    global depthData, isDepthReady, dist 
    rospy.init_node('depth_example', anonymous=True)
    rospy.Subscriber("/camera/depth/image", Image, depthCallback, queue_size=10)
    rospy.Subscriber("/blobs", Blobs, updateBlobsInfo)
    rospy.Subscriber("/camera/rgb/image_color", Image, updateColorImage)
    rospy.Subscriber('/odom', Odometry, odomCallback)
    
    odomReset.publish(Empty())
    #remember to record odom somewhere because if minor adjustments are needed (like slightly turning left etc) then we should do that before our robot starts from a wrong direction. 

    move(dist)
    while not isDepthReady:
        pass

    while not rospy.is_shutdown():
        step = depthData.step
        midX = 320
        midY = 240
        offset = (240 * step) + (320 * 4)
        (dist,) = unpack('f', depthData.data[offset] + depthData.data[offset+1] + depthData.data[offset+2] + depthData.data[offset+3])
	
        print "Distance: %f" % dist

if __name__ == '__main__':
    main()
