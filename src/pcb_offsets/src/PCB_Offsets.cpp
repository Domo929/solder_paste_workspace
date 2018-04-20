#include <iostream>
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"

#include "geometry_msgs/Pose.h"
#include "std_msgs/Bool.h"

#include "tf/tf.h"

float marker_size = 10.0;
int marker_id = 0;

int camera_index = 0;
std::string path_to_calibration = "/home/djcupo/solder_paste_workspace/src/pcb_offsets/camera_calib/Dynamics_Cam_Temp.xml";

bool start_calibration = false;

void start_callback(const std_msgs::Bool::ConstPtr& msg){
    std::cout << "Got start message" << std::endl;
    start_calibration = true;
}

int main(int argc, char* argv[]) {

    std::cout << "Loading" << std::endl;

    ros::init(argc, argv, "PCB_Offsets_Node");

    ros::NodeHandle nh;

    ros::Subscriber start_sub = nh.subscribe("/start_offset", 5, start_callback);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose>("/PCB_Offset_Pose", 5);

    const cv::Ptr<cv::aruco::Dictionary> dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();

    cv::Mat cam_mat;
    cv::Mat cam_dst;

    cv::FileStorage fs;

    fs.open(path_to_calibration, cv::FileStorage::READ);

    if(!fs.isOpened()){
        return -1;
    } else {
        fs["camera_matrix"] >> cam_mat;
        fs["distortion_coefficients"] >> cam_dst;
        fs.release();
    }

    std::cout << "Camera File Loaded" << std::endl;

    cv::VideoCapture vc = cv::VideoCapture(camera_index);

    if(!vc.isOpened()){
        return -2;
    }

    std::cout << "Video Stream Opened" << std::endl;

    ros::Rate rate(10);

    while(vc.grab() && ros::ok()){
        ros::spinOnce();
        if(start_calibration){
            std::cout << "Starting Calibration" << std::endl;
            cv::Mat img;

            vc.retrieve(img);

            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f> > corners, rejected;
            std::cout << "About to detect markers" << std::endl;
            cv::aruco::detectMarkers(img, dict, corners, ids, detectorParams, rejected);
            std::cout << "detected markers" << std::endl;

            if (!ids.empty()){

                std::cout << "Found some tags: " << ids.size() << std::endl;
                std::vector<cv::Vec3d> rvec, tvec;
                std::cout << "Estimating Pose" << std::endl;
                cv::aruco::estimatePoseSingleMarkers(corners, marker_size, cam_mat, cam_dst, rvec, tvec);

                for(int i = 0; i < ids.size(); i++){

                    if(ids[i] == marker_id){
                        std::cout << "Found OUR Tag" << std::endl;
                        geometry_msgs::Pose pose = geometry_msgs::Pose();

                        pose.position.x = tvec[i][0];
                        pose.position.y = tvec[i][1];
                        pose.position.z = tvec[i][2];

                        tf::Quaternion q = tf::createQuaternionFromRPY(rvec
                                [i][0], rvec[i][1], rvec[i][2]);

                        pose.orientation.x = q.x();
                        pose.orientation.y = q.y();
                        pose.orientation.z = q.z();
                        pose.orientation.w = q.w();

                        pose_pub.publish(pose);

                        start_calibration = false;
                    }
                }
            } else {
                std::cout << "Did not find a marker" << std::endl;
            }
        }
        rate.sleep();
    }

    return 0;
}