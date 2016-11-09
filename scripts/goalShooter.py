#!/usr/bin/env python

import roslib
import rospy
import cv2
import copy
import time
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from cmvision.msg import Blobs, Blob
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

colorImage = Image()
isColorImageReady = False
blobsInfo = Blobs()
isBlobsInfoReady = False
command = Twist()
command.linear.x = 0 
command.angular.z = 0
pub = rospy.Publisher("kobuki_command", Twist, queue_size=10)
odomReset = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)

positionX = 0.0
positionY = 0.0
degree = 0.0
tempDeg = 0.0
velocity = 0.0
distOffset = 0

a0 = 0
a1 = 0
b0 = 0
b1 = 0
l0 = 0.5
l1 = 0
l2 = 0
c0 = 0
ballDir = "R"

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

def updateBlobsInfo(data):
    global blobsInfo, isBlobsInfoReady
    blobsInfo = data
    isBlobsInfoReady = True

def mergeBlobs(blobsInfo):
    x = 0
    y = 0
    left = 0
    right = 0
    top = 0
    bottom = 0
    area = 0

    overlapsFound = 1
    blobs = blobsInfo.blobs

    for b in blobs:

        blobs2 = copy.deepcopy(blobs)

        for b2 in blobs2:
            if(b.name == b2.name and b != b2):
                overlapping = 0

                blobx = b2.left
                while blobx < b2.right and overlapping == 0:
                    bloby = b2.top
                    while bloby < b2.bottom and overlapping == 0:
                        if blobx >= b.left and blobx <= b.right and bloby >= b.top and bloby <= b.bottom:
                            overlapping = 1
                        bloby = bloby + 1
                    blobx = blobx + 1

                if overlapping == 1:
                    x = b.x + (b2.x * b2.area)
                    y = b.y + (b2.y * b2.area)
                    left = b.left  + (b2.left * b2.area)
                    right = b.right + (b2.right * b2.area)
                    top = b.top + (b2.top * b2.area)
                    bottom = b.bottom + (b2.bottom * b2.area)
                    area = b.area + b2.area

                    result = Blob()
                    result.x = x / area
                    result.y = y / area
                    result.left = left / area
                    result.right = right / area
                    result.top = top / area
                    result.bottom = bottom / area
                    result.area = x * y

                    b = result
                    blobs.remove(b2)

    return blobs

def cleanUp():
    global command
    command.linear.x = 0.0
    command.angular.z = 0.0
    pub.publish(command)
    odomReset.publish(Empty())
    rospy.sleep(1)

def findBallAndGoal(direction, angleNumber):
    global blobsInfo, a0, a1, b0, b1, l0, l1, c0, ballDir
    ballBlob = 0
    goalBlob = 0
    blobsCopy = copy.deepcopy(blobsInfo)
    blobs = mergeBlobs(blobsCopy)
    #print blobs
    if len(blobs) > 0:
        for b in blobs:
            if ballBlob == 0:
                if b.name == 'BallRed':
                    ballBlob = b
            elif b.area > ballBlob.area and b.name == 'BallRed':
                ballBlob = b


    if ballBlob != 0:
	if ballBlob.x > 300 and ballBlob.x < 340:
            if angleNumber == 0:
                if a0 == 0:
		    if direction == "R":
                        a0 = 180 - math.fabs(degree)
                    else:
                        a0 = math.fabs(degree)

                    if a0 > 90:
                        ballDir = "L"

                    print "found ball angle 0: " + str(a0)
            elif angleNumber == 1:
                if a1 == 0:
                    if direction == "R":
                        a1 = 180 - math.fabs(degree)
                    else:
                        a1 = math.fabs(degree)
                    print "found ball angle 1: " + str(a1)
    

    if len(blobs) > 0:
        for b in blobs:
            if b.name == 'GoalYellow':
                for b2 in blobs:
                    if b2.name == 'GoalPink':
                        if b2.left > b.left and b2.right < b.right and b2.top > b.top and b2.bottom < b.bottom:
                            if goalBlob == 0:
                                goalBlob = b
                            elif b.area > goalBlob.area:
                                goalBlob = b
    if goalBlob != 0:
        if goalBlob.x > 250 and goalBlob.x < 390:
            if angleNumber == 0:
                if b0 == 0:
                    if direction == "R":
                        b0 = 180 - math.fabs(degree)
                    else:
                        b0 = math.fabs(degree)
                    print "found goal angle 0: " + str(b0)
            elif angleNumber == 1:
                if b1 == 0:
                    if direction == "R":
                        b1 = 180 - math.fabs(degree)
                    else:
                        b1 = math.fabs(degree)
                    print "found goal angle 1: " + str(b1)

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

def sin(deg):
    rad = math.radians(deg)
    return math.sin(rad)

def cos(deg):
    rad = math.radians(deg)
    return math.cos(rad)

def main():
    global colorImage, isColorImageReady, blobsInfo, isBlobsInfoReady, a0, a1, b0, b1, l0, l1, l2, c0, ballBlob, goalBlob, ballDir
    rospy.init_node('showBlobs', anonymous=True)
    rospy.Subscriber("/blobs", Blobs, updateBlobsInfo)
    rospy.Subscriber("/v4l/camera/image_raw", Image, updateColorImage)
    rospy.Subscriber('/odom', Odometry, odomCallback)
    rospy.on_shutdown(cleanUp)

    while not rospy.is_shutdown() and (not isBlobsInfoReady or not isColorImageReady):
        pass

    odomReset.publish(Empty())
    rospy.sleep(1)

    # find ball and goal from start
    runCommand("R", 0.3, 90, 3)
    runCommand("L", 0.3, 180, 0)
    while a0 == 0 or b0 == 0:
        runCommand("R", 0.3, 180, 0)
        runCommand("L", 0.3, 180, 0)
	
    #move away l0 m
    runCommand("F", 0.3, l0, 3)

    #find ball and goal from here
    runCommand("R", 0.3, 180, 1)
        
    while a1 == 0 or b1 == 0:
        runCommand("L", 0.3, 180, 1)
        runCommand("R", 0.3, 180, 1)
        
    if a0 != 0 and a1 != 0 and b0 != 0 and b1 != 0: 
        print "calculating.."
        #calculate distances
        x0 = 180 - b1 - (180 - b0)
        x1 = (l0 * sin(b1))/sin(x0)
        x2 = (l0 * sin(180 - a0))/sin(180 - (180 - a0) - a1)
        x3 = (l0 * sin(180 - b0))/sin(x0)
        x5 = math.sqrt((x2 * x2) + (x3 * x3) - (2*x2*x3*cos(b1-a1)))
        x4 = math.degrees(math.asin((x2*sin(b1-a1))/x5))
        
        c0 = 180 - x4 - b1
        l1 = (x1*sin(x4-x0))/sin(c0)
        l2 = ((sin(b1)*(l0 + l1))/sin(x4)) - x5

        print x0
        print x1
        print x2
        print x3
        print x4
        print x5
        print c0
        print l1
        print l2
        print ballDir
        #move l0 + l1 right
        l = l0+l1

        if ballDir == "R":
            l = l + 0.2
        else:
            l = l - 0.2
        print l
        if l >= 0:
            runCommand("F", 0.3, l, 3)
        else:
            runCommand("B", 0.3, math.fabs(l), 3)

        #turn 180 - c0 left
        runCommand("L", 0.3, (180 - c0), 1)

        #go forward l2 and maybe hit the ball
        runCommand("F", 1, l2 + 0.35, 3)
    else:
        print "Angles not found!!"
 
if __name__ == '__main__':
    main()
