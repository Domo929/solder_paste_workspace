import argparse

import cv2
import cv2.aruco as aruco
import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

global start_calib
start_calib = False

# TODO print and determine size
marker_size = 1

tag_index = 0


def start_callback(data):
    global start_calib
    start_calib = data


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Handles the PCB Offsets')

    parser.add_argument('index', type=int, required=True,
                        help='The /dev/video* index')
    parser.add_argument('path_to_calibration_file', type=str,
                        help='The path to the OpenCV Calibration file for the camera')

    args = vars(parser.parse_args())

    rospy.init_node('PCB_Offsets_Node', anonymous=False)

    pose_pub = rospy.Publisher('/PCB_Offset', Pose, queue_size=1)

    start_sub = rospy.Subscriber('/Start_Offset', Bool, start_callback)

    aruco_dict = aruco.Dictionary_get(aruco.DICT_7X7_1000)
    parameters = aruco.DetectorParameters_create()

    cam_mat = None
    cam_dst = None

    fs = cv2.FileStorage()

    fs.open(args['path_to_calibration_file'])

    if fs.isOpened():
        cam_mat = fs.getNode('CameraMatrix').mat()
        cam_dst = fs.getNode('Distortion_Coefficients').mat()
        fs.release()
    else:
        print('Could not open the calibration file. Is the path correct?')
        exit(-1)

    cap = cv2.VideoCapture(args['index'])

    while not rospy.is_shutdown():
        if start_calib:

            ret, frame = cap.read()

            if ret:
                frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                corners, ids, _ = aruco.detectMarkers(frame_gray, aruco_dict,
                                                      parameters=parameters, cameraMatrix=cam_mat, distCoeff=cam_dst)
                if ids and len(ids) > 0:
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, cam_mat, cam_dst)

                    for index in range(len(ids)):
                        if ids[index] == tag_index:
                            Q = tf.transformations.quaternion_from_euler(
                                rvec[index][0], rvec[index][1], rvec[index][2])

                            pose = Pose()

                            pose.position.x = tvec[index][0]
                            pose.position.y = tvec[index][1]
                            pose.position.z = tvec[index][2]

                            pose.orientation.x = Q.x
                            pose.orientation.y = Q.y
                            pose.orientation.z = Q.z
                            pose.orientation.w = Q.w

                            pose_pub.publish(pose)

                else:
                    print("Did not find any tags")
            else:
                print("Error obtaining frame")
