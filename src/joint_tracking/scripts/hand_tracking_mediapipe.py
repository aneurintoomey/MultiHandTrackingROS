#!/usr/bin/env python

from __future__ import print_function

from joint_tracking.srv import MediapipeTracker, MediapipeTrackerResponse
from joint_tracking.msg import Hand
import cv2
import rospy
import ros_numpy

import mediapipe as mp
import numpy
import os

def handleService(req):
    rospy.loginfo("Service Requested")

    __location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))


    BaseOptions = mp.tasks.BaseOptions
    HandLandmarker = mp.tasks.vision.HandLandmarker
    HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
    VisionRunningMode = mp.tasks.vision.RunningMode

    options = HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=os.path.join(__location__, 'hand_landmarker.task')),
        running_mode=VisionRunningMode.IMAGE,
        num_hands=2)

    #Reverse from BGR format to RGB
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(ros_numpy.numpify(req.image), cv2.COLOR_BGR2RGB))

    landmarker = HandLandmarker.create_from_options(options)
    hand_landmarker_result = landmarker.detect(mp_image)

    #Formats response to be sent
    retval = MediapipeTrackerResponse()
    retval.left = Hand()
    retval.right = Hand()
    retval.left.type = "Left"
    retval.right.type = "Right"

    for index in range(len(hand_landmarker_result.hand_landmarks)):
        hand_landmarks = hand_landmarker_result.hand_landmarks[index]
        handedness = hand_landmarker_result.handedness[index]

        u = list()
        v = list()
        z = list()

        for landmark in hand_landmarks:
            u.append(landmark.x)
            v.append(landmark.y)
            z.append(landmark.z)

        rospy.loginfo(handedness[0].display_name)
        
        if(handedness[0].display_name == "Left"):
            retval.left.u = u
            retval.left.v = v
            retval.left.z = z

        elif(handedness[0].display_name == "Right"):
            rospy.loginfo(u)
            retval.right.u = u
            rospy.loginfo(retval.right.u)
            retval.right.v = v
            retval.right.z = z

    rospy.loginfo("Service Responded")

    return retval

if __name__ == "__main__":
    rospy.init_node('MediapipeTrackerServer')
    s = rospy.Service('MediapipeTracker', MediapipeTracker, handleService)
    rospy.loginfo("Hand Tracker Service Started")
    rospy.spin()