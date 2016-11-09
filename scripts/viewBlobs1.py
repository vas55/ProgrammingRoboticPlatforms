#!/usr/bin/env python

import roslib
import rospy
import cv2
import copy
import time
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from cmvision.msg import Blobs, Blob
from geometry_msgs.msg import Twist

colorImage = Image()
isColorImageReady = False
blobsInfo = Blobs()
isBlobsInfoReady = False
targetCommand = Twist()
currentCommand = Twist()
currentCommand.linear.x = 0 
currentCommand.angular.z = 0
pub = rospy.Publisher("kobuki_command", Twist, queue_size=10)
kp = 0.0035
kd = 0.000
oldTime = 0
oldErr = 0


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

def goToBlob(blobSet, blob):
    global command, pub, kp, oldTime, oldErr
    x = 0
    z = 0
    newTime = time.time()
    newErr = (320 - blob.x)
    de = oldErr - newErr
    dt = oldTime - newTime
    print ("blobSet in goTOBLOB", blobSet)
    if oldTime == 0: 
        de = 0
        dt = 1

    if blobSet == 1:
        x = 0.2
        z = newErr*kp + kd*(de/dt)
    
    currentCommand.linear.x = x
    currentCommand.angular.z = z

    oldTime = newTime
    oldErr = newErr
    
    pub.publish(currentCommand)

def cleanUp():
    global currentCommand
    currentCommand.linear.x = 0.0
    currentCommand.angular.z = 0.0
    pub.publish(currentCommand)
    rospy.sleep(1)

def main():
    global colorImage, isColorImageReady, blobsInfo, isBlobsInfoReady
    rospy.init_node('showBlobs', anonymous=True)
    rospy.Subscriber("/blobs", Blobs, updateBlobsInfo)
    rospy.Subscriber("/v4l/camera/image_raw", Image, updateColorImage)
    rospy.on_shutdown(cleanUp)

    while not rospy.is_shutdown() and (not isBlobsInfoReady or not isColorImageReady):
        pass

    while not rospy.is_shutdown():
        blobsCopy = copy.deepcopy(blobsInfo)
        blobs = mergeBlobs(blobsCopy)
        blobSet = 0

        if len(blobs) > 0:
            biggestBlob = blobs[0]

            for b in blobs:
                if blobSet == 0:
                    if b.name == 'LineGreen':
                        biggestBlob = b
                        blobSet = 1
                elif b.area > biggestBlob.area and b.name == 'LineGreen':
                    biggestBlob = b

            goToBlob(blobSet, biggestBlob)
            # rospy.sleep(0.01)

    currentCommand.linear.x = 0
    currentCommand.angular.z = 0
    pub.publish(currentCommand)
 
if __name__ == '__main__':
    main()
