#!/usr/bin/env python

from __future__ import print_function

import cv2
import rospy
import ros_numpy

import mediapipe as mp
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2

import numpy
import os

from joint_tracking.srv import MediapipeTracker, MediapipeTrackerResponse
from joint_tracking.msg import Hand

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

    #Reverse image format from BGR format to RGB and load it
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

        hand_landmarker_result.handedness[index] += str(index)

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


#draw_landmarks sourced from mediapipe documentation
def draw_landmarks_on_image(rgb_image, detection_result):
  hand_landmarks_list = detection_result.hand_landmarks
  handedness_list = detection_result.handedness
  annotated_image = numpy.copy(rgb_image)

  # Loop through the detected hands to visualize.
  for idx in range(len(hand_landmarks_list)):
    hand_landmarks = hand_landmarks_list[idx]
    handedness = handedness_list[idx]

    # Draw the hand landmarks.
    hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
    hand_landmarks_proto.landmark.extend([
      landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in hand_landmarks
    ])
    solutions.drawing_utils.draw_landmarks(
      annotated_image,
      hand_landmarks_proto,
      solutions.hands.HAND_CONNECTIONS,
      solutions.drawing_styles.get_default_hand_landmarks_style(),
      solutions.drawing_styles.get_default_hand_connections_style())

    # Get the top left corner of the detected hand's bounding box.
    height, width, _ = annotated_image.shape
    x_coordinates = [landmark.x for landmark in hand_landmarks]
    y_coordinates = [landmark.y for landmark in hand_landmarks]
    text_x = int(min(x_coordinates) * width)
    text_y = int(min(y_coordinates) * height) - 10

    # Draw handedness (left or right hand) on the image.
    cv2.putText(annotated_image, f"{handedness[0].category_name}",
                (text_x, text_y), cv2.FONT_HERSHEY_DUPLEX,
                1, (88, 205, 54), 1, cv2.LINE_AA)

if __name__ == "__main__":
    rospy.init_node('MediapipeTrackerServer')
    s = rospy.Service('MediapipeTracker', MediapipeTracker, handleService)
    rospy.loginfo("Hand Tracker Service Started")
    rospy.spin()