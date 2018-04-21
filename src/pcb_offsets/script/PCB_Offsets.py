#!/usr/bin/env python

# Python libraries
import argparse
import numpy as np

# OpenCV and ArUco libraries
import cv2
import cv2.aruco as aruco

# ROS Libraries
import rospy
import tf

# ROS messages
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

# The marker size in meters
marker_size = 0.01

# The index of the tag to search for
tag_index = 0

# Message callback that sets the start calibration paramter to True
def start_callback(data):
    global should_start_calib
    should_start_calib = True
    print("Got a start message")


if __name__ == '__main__':
    global should_start_calib

    # Set up the parser for the camera index and camera calibration file location
    parser = argparse.ArgumentParser(description='Handles the PCB Offsets')

    parser.add_argument('index', type=int,
                        help='The /dev/video* index')
    parser.add_argument('path_to_calibration_file', type=str,
                        help='The path to the OpenCV Calibration file for the camera')

    parser.add_argument('-t', '--tag-id', type=int, help='If using a tag with ID other than 0, specify here')

    args = vars(parser.parse_args())

    if bool(args['--tag-id']):


    # Initialize the ROS node
    rospy.init_node('PCB_Offsets_Node', anonymous=False)

    # Set up the Pose publisher
    pose_pub = rospy.Publisher('/PCB_Offset', Pose, queue_size=1)

    # Set up the start subscriber
    should_start_calib = False
    start_sub = rospy.Subscriber('/start_offset', Bool, start_callback)

    # Create the Aruco Dictionary object and Detector Parameters
    # Currently we use a 4x4 tag
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()

    # Initialize the cmaera matrix and distorsion images to the generic, empty numpy arrays
    cam_mat = np.zeros((1,1), np.uint8)
    cam_dst = np.zeros((1,1), np.uint8)

    # Create an OpenCV File object to parse the Camera Calibration files correctly
    fs = cv2.FileStorage()

    # Open from the path given
    fs.open(args['path_to_calibration_file'], cv2.FILE_STORAGE_READ)

    # Error check to make sure the file exists/can be read
    # If it can, it loads the two pieces of information
    if fs.isOpened():
        cam_mat = fs.getNode('Camera_Matrix').mat()
        cam_dst = fs.getNode('Distortion_Coefficients').mat()
        fs.release()
    else:
        print('Could not open the calibration file. Is the path correct?')
        exit(-1)
    
    # Create the Videocapture object, opening the camera index specified
    cap = cv2.VideoCapture(args['index'])

    # Set up a 5HZ rate to not spam the system
    rate = rospy.Rate(5)

    # Loop until we kill you
    while not rospy.is_shutdown():

        global should_start_calib

        # If we have received the command to start calibrating:
        if should_start_calib:
            print('Starting')

            # Get a frame and success flag from the video capture object
            ret, frame = cap.read()

            # If the frame is valid:
            if ret:

                # Convert the frame to grayscale
                frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                print('Looking for tags')

                # Search that image for aruco tags matching the dictionary object. 
                # Returns locations of corners, the tags ID, and a list of rejected points (which we ignore)
                corners, ids, _ = aruco.detectMarkers(frame_gray, aruco_dict, parameters=parameters)

                # If we found IDs, proceed, otherwise we access an empty list and break shit
                if ids is not None and len(ids) > 0:
                    print('Found tag')

                    # Use the known marker size, corner location and camera parameters, determine the offsets
                    r_vec, t_vec, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, cam_mat, cam_dst)

                    # Iterate over the ids, to find the one that matches our board
                    for index in range(len(ids)):

                        # If we found the correct tag:
                        if ids[index] == tag_index:
                            
                            # Convert the Euler angle to Quaternion
                            Q = tf.transformations.quaternion_from_euler(
                                r_vec[index][0][0], r_vec[index][0][1], r_vec[index][0][2])

                            # Make the pose message
                            pose = Pose()

                            # Fill the pose message
                            pose.position.x = t_vec[index][0][0]
                            pose.position.y = t_vec[index][0][1]
                            pose.position.z = t_vec[index][0][2]

                            pose.orientation.x = Q[0]
                            pose.orientation.y = Q[1]
                            pose.orientation.z = Q[2]
                            pose.orientation.w = Q[3]

                            pose_pub.publish(pose)

                            should_start_calib = False

                # Let use know we did not find any tags
                else:
                    print("Did not find any tags")
            # Report issues with the camera
            else:
                print("Error obtaining frame")
    # Keep the loop rate limited
    rate.sleep()