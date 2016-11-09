#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import Image
from struct import *
from graph import Graph
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Empty

pub = rospy.Publisher("kobuki_command", Twist, queue_size=10)
odomReset = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
command = Twist()

positionX = 0.0
positionY = 0.0
degree = 0.0
velocity = 0.0
distOffset = 0

depthData = Image()
isDepthReady = False
m = Graph()

currentNode = 'start'
targetNode = ''
directionNumber = 0

leftDist = 0
rightDist = 0

collision = False

def depthCallback(data):
    global depthData, isDepthReady, collision, leftDist, rightDist
    depthData = data
    step = depthData.step
    midX = 320
    midY = 240
    
    leftOffset = (midY * step) + (40 * 4)
    rightOffset = (midY * step) + (600 * 4)
    (leftDist,) = unpack('f', depthData.data[leftOffset] + depthData.data[leftOffset+1] + depthData.data[leftOffset+2] + depthData.data[leftOffset+3])
    (rightDist,) = unpack('f', depthData.data[rightOffset] + depthData.data[rightOffset+1] + depthData.data[rightOffset+2] + depthData.data[rightOffset+3])
    
    collision = False

    for x in range(280, 360):
        offset = (midY * step) + (x * 4)
        (dist,) = unpack('f', depthData.data[offset] + depthData.data[offset+1] + depthData.data[offset+2] + depthData.data[offset+3])
        if(not math.isnan(dist) and dist < 0.5):
            collision = True

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

def runCommand(direction, maxSpeed, distance):
    global pub, command, positionX, positionY, degree, tempDeg, velocity, distOffset, leftDist, rightDist, collision

    if direction == "R" or direction == "L":
        tempDeg = abs(degree)
        bigTurn = 0

        if distance >= 360:
            distance = 359

        # adjust for error
        distance = distance - 5

        while tempDeg < distance:       
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
        # change feet to meters
        distance = distance*0.3048

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
                    tempSpeed = tempSpeed - 0.002

            if(direction == "B"):
                tempSpeed = tempSpeed*-1
    
            command.linear.x = tempSpeed

            if(not math.isnan(leftDist) and not math.isnan(rightDist) and (distance - abs(positionX)) > 2.0 and abs(positionX) > 0.5):
                error = leftDist - rightDist
                command.angular.z = error*0.5
            else:
                command.angular.z = 0

            if collision:
                command.linear.x = 0.0
                command.angular.z = 0.0

            pub.publish(command)
            rospy.sleep(0.01)

    command.linear.x = 0.0
    command.angular.z = 0.0

    pub.publish(command)
    odomReset.publish(Empty())
    rospy.sleep(0.5)

def initMap():
    # the current direction number is used to determine which way to turn at turn nodes
    # ex. R-L-90-1, if direction = 0 we will turn right 90 degrees, if direction = 1
    # we will turn left 90 degrees.  Essentially <turnDir1>-<turnDir2><degree><counter>.
    # the counter is just to make the node unique among all turn nodes

    m.add('start2', '5801', {
        'direction': 0,
        'distance': 13
    });
    m.add('5801', 'start2', {
        'direction': 1,
        'distance': 13
    });
    #=========================
    m.add('5801', 'L-R-90-1', {
        'direction': 0,
        'distance': 27
    });
    m.add('L-R-90-1', '5801', {
        'direction': 1,
        'distance': 27
    });
    #=========================
    m.add('L-R-90-1', '5414', {
        'direction': 0,
        'distance': 15.5
    });
    m.add('5414', 'L-R-90-1', {
        'direction': 1,
        'distance': 15.5
    });
    #=========================
    m.add('5414', '5412', {
        'direction': 0,
        'distance': 5.5
    });
    m.add('5412', '5414', {
        'direction': 1,
        'distance': 5.5
    });
    #=========================
    m.add('5412', '5411', {
        'direction': 0,
        'distance': 14
    });
    m.add('5411', '5412', {
        'direction': 1,
        'distance': 14
    });
    #=========================
    m.add('5411', '5409', {
        'direction': 0,
        'distance': 6
    });
    m.add('5409', '5411', {
        'direction': 1,
        'distance': 6
    });
    #=========================
    m.add('5409', '5406', {
        'direction': 0,
        'distance': 14
    });
    m.add('5406', '5409', {
        'direction': 1,
        'distance': 14
    });
    #=========================
    m.add('5406', '5404', {
        'direction': 0,
        'distance': 6
    });
    m.add('5404', '5406', {
        'direction': 1,
        'distance': 6
    });
    #=========================
    m.add('5404', '5403', {
        'direction': 0,
        'distance': 14
    });
    m.add('5403', '5404', {
        'direction': 1,
        'distance': 14
    });
    #=========================
    m.add('5403', 'L-R-90-2', {
        'direction': 0,
        'distance': 2
    });
    m.add('L-R-90-2', '5403', {
        'direction': 1,
        'distance': 2
    });
    #=========================
    m.add('L-R-90-2', '5329', {
        'direction': 0,
        'distance': 10.5
    });
    m.add('5329', 'L-R-90-2', {
        'direction': 1,
        'distance': 10.5
    });
    #=========================
    m.add('5329', '5327', {
        'direction': 0,
        'distance': 5.5
    });
    m.add('5327', '5329', {
        'direction': 1,
        'distance': 5.5
    });
    #=========================
    m.add('5327', '5091', {
        'direction': 0,
        'distance': 6
    });
    m.add('5091', '5327', {
        'direction': 1,
        'distance': 6
    });
    #=========================
    m.add('5091', 'L-R-90-3', { # L-R-90-3 is also room 5325
        'direction': 0,
        'distance': 7
    });
    m.add('L-R-90-3', '5091', {
        'direction': 1,
        'distance': 7
    });
    #=========================
    m.add('L-R-90-3', 'R-L-90-1', {
        'direction': 0,
        'distance': 13
    });
    m.add('R-L-90-1', 'L-R-90-3', {
        'direction': 1,
        'distance': 13
    });
    #=========================
    m.add('R-L-90-1', '5321', {
        'direction': 0,
        'distance': 6
    });
    m.add('5321', 'R-L-90-1', {
        'direction': 1,
        'distance': 6
    });
    #=========================
    m.add('5321', '5324', {
        'direction': 0,
        'distance': 2
    });
    m.add('5324', '5321', {
        'direction': 1,
        'distance': 2
    });
    #=========================
    m.add('5324', '5519', {
        'direction': 0,
        'distance': 17
    });
    m.add('5519', '5324', {
        'direction': 1,
        'distance': 17
    });
    #=========================
    m.add('5519', '5517', {
        'direction': 0,
        'distance': 6
    });
    m.add('5517', '5519', {
        'direction': 1,
        'distance': 6
    });
    #=========================
    m.add('5517', 'L-R-90-4', {
        'direction': 0,
        'distance': 5
    });
    m.add('L-R-90-4', '5517', {
        'direction': 1,
        'distance': 5
    });
    #=========================
    m.add('5517', 'L-R-90-4', {
        'direction': 0,
        'distance': 5
    });
    m.add('L-R-90-4', '5517', {
        'direction': 1,
        'distance': 5
    });
    #=========================
    m.add('L-R-90-4', '5502', {
        'direction': 0,
        'distance': 5
    });
    m.add('5502', 'L-R-90-4', {
        'direction': 1,
        'distance': 5
    });
    #=========================
    m.add('5502', '5501', {
        'direction': 0,
        'distance': 8
    });
    m.add('5501', '5502', {
        'direction': 1,
        'distance': 8
    });
    #=========================
    m.add('5501', '5503', {
        'direction': 0,
        'distance': 5
    });
    m.add('5503', '5501', {
        'direction': 1,
        'distance': 5
    });
    #=========================
    m.add('5503', '5505', {
        'direction': 0,
        'distance': 13.5
    });
    m.add('5505', '5503', {
        'direction': 1,
        'distance': 13.5
    });
    #=========================
    m.add('5505', '5506', {
        'direction': 0,
        'distance': 8
    });
    m.add('5506', '5505', {
        'direction': 1,
        'distance': 8
    });
    #=========================
    m.add('5506', 'L-R-90-5', {
        'direction': 0,
        'distance': 23
    });
    m.add('L-R-90-5', '5506', {
        'direction': 1,
        'distance': 23
    });
    #=========================
    m.add('L-R-90-5', 'start2', {
        'direction': 0,
        'distance': 24
    });
    m.add('start2', 'L-R-90-5', {
        'direction': 1,
        'distance': 24
    });


def main():
    global depthData, isDepthReady, currentNode, targetNode, directionNumber, m
    rospy.init_node('depth_example', anonymous=True)
    rospy.Subscriber("/camera/depth/image", Image, depthCallback, queue_size=10)
    rospy.Subscriber("/camera/rgb/image_color", Image, updateColorImage)
    rospy.Subscriber('/odom', Odometry, odomCallback)
    
    initMap()
    odomReset.publish(Empty())
    goHome = False
    #remember to record odom somewhere because if minor adjustments are needed (like slightly turning left etc) then we should do that before our robot starts from a wrong direction. 
    
    while not isDepthReady:
        pass

    currentNode = raw_input('Where am I? ')

    if(currentNode != 'start'):
        directionNumber = int(raw_input('Current direction number? '))


    while not rospy.is_shutdown() and not goHome:
        targetNode = raw_input('Where to? ')
        newDirectionNumber = int(raw_input('Direction number? '))

        if(currentNode == 'start'):
            runCommand("F", 0.5, 24)
            runCommand("R", 0.5, 90)
            currentNode = 'start2'

        if(targetNode == 'start'):
            goHome = True
            targetNode = 'start2'

        path = m.find_path(currentNode, targetNode, newDirectionNumber)
        path.pop(0)
        print path
        print m.get_info(currentNode, path[0])

        if(newDirectionNumber != directionNumber):
            runCommand("R", 0.5, 180)

        directionNumber = newDirectionNumber
 
        for node in path:
            runCommand("F", 0.5, m.get_info(currentNode, node)['distance'])

            if "R" in node or "L" in node:
                turnInfo = node.split('-')
                turnDir = turnInfo[directionNumber]
                runCommand(turnDir, 0.5, float(turnInfo[2]))

            currentNode = node

        if(goHome):
            if directionNumber == 0:
                runCommand("R", 0.5, 90)
            else:
                runCommand("L", 0.5, 90)

            runCommand("F", 0.5, 24)


if __name__ == '__main__':
    main()
