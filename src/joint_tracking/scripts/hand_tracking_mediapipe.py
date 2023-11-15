#!/usr/bin/env python

from __future__ import print_function

from joint_tracking.srv import mediapipeTracker,mediapipeTrackerResponse
from joint_tracking.msg import Hand
from cv_bridge import CvBridge
import rospy

import mediapipe as mp
import numpy

def processLandmarks(hand_landmarker_result):
    retval = mediapipeTrackerResponse()

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
        
        if(handedness[0] == "Left"):
            retval.left.u = tuple(u)
            retval.left.v = tuple(v)
            retval.left.z = tuple(z)

        elif(handedness[0] == "Right"):
            retval.right.u = tuple(u)
            retval.right.v = tuple(v)
            retval.right.z = tuple(z)
    
    return retval

def handleService(req):
    BaseOptions = mp.tasks.BaseOptions
    HandLandmarker = mp.tasks.vision.HandLandmarker
    HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
    VisionRunningMode = mp.tasks.vision.RunningMode

    options = HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path='scripts/hand_landmarker.task'),
        running_mode=VisionRunningMode.IMAGE,
        num_hands=2)
    
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(req.image, desired_encoding='passthrough')
    
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=numpy.asarray(cv_image))

    landmarker = HandLandmarker.create_from_options(options)

    return processLandmarks(landmarker.detect(mp_image))

def startServer():
    rospy.init_node('mediapipeTrackerServer')
    s = rospy.Service('mediapipeTracker', mediapipeTracker, handleService)
    rospy.spin()

if __name__ == "__main__":
    startServer()