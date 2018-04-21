#!/usr/bin/env python

import argparse

import cv2
import cv2.aruco as aruco
import rospy
import tf
import numpy as np
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

# TODO print and determine size
marker_size = 0.01

tag_index = 0

def start_callback(data):
    global should_start_calib
    should_start_calib = True
    print("Got a start message")


if __name__ == '__main__':
    global should_start_calib

    parser = argparse.ArgumentParser(description='Handles the PCB Offsets')

    parser.add_argument('index', type=int,
                        help='The /dev/video* index')
    parser.add_argument('path_to_calibration_file', type=str,
                        help='The path to the OpenCV Calibration file for the camera')

    args = vars(parser.parse_args())

    rospy.init_node('PCB_Offsets_Node', anonymous=False)

    pose_pub = rospy.Publisher('/PCB_Offset', Pose, queue_size=1)

    should_start_calib = False
    start_sub = rospy.Subscriber('/start_offset', Bool, start_callback)

    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()

    cam_mat = np.zeros((1,1), np.uint8)
    cam_dst = np.zeros((1,1), np.uint8)

    fs = cv2.FileStorage()

    fs.open(args['path_to_calibration_file'], cv2.FILE_STORAGE_READ)

    if fs.isOpened():
        cam_mat = fs.getNode('Camera_Matrix').mat()
        cam_dst = fs.getNode('Distortion_Coefficients').mat()
        fs.release()
    else:
        print('Could not open the calibration file. Is the path correct?')
        exit(-1)

    cap = cv2.VideoCapture(args['index'])

    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        # print('looping')
        global should_start_calib
        if should_start_calib:
            print('Starting')
            ret, frame = cap.read()

            if ret:
                frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                print('Looking for tags')
                corners, ids, _ = aruco.detectMarkers(frame_gray, aruco_dict, parameters=parameters)
                if ids is not None and len(ids) > 0:
                    print('Found tag')
                    r_vec, t_vec, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, cam_mat, cam_dst)

                    for index in range(len(ids)):
                        if ids[index] == tag_index:
                            print(r_vec)
                            Q = tf.transformations.quaternion_from_euler(
                                r_vec[index][0][0], r_vec[index][0][1], r_vec[index][0][2])

                            pose = Pose()

                            pose.position.x = t_vec[index][0][0]
                            pose.position.y = t_vec[index][0][1]
                            pose.position.z = t_vec[index][0][2]

                            pose.orientation.x = Q[0]
                            pose.orientation.y = Q[1]
                            pose.orientation.z = Q[2]
                            pose.orientation.w = Q[3]

                            pose_pub.publish(pose)

                            should_start_calib = False

                else:
                    print("Did not find any tags")
            else:
                print("Error obtaining frame")
    rate.sleep()