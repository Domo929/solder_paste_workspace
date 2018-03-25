//
// Created by yan on 2/4/18.
//


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ApplyJointEffort.h"
#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "talker");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::ServiceClient joint_effort_client = n.serviceClient<gazebo_msgs::ApplyJointEffort>("gazebo/apply_joint_effort");
    gazebo_msgs::ApplyJointEffort srv;

    srv.request.duration.nsec = 100000;

    srv.request.effort = 10000;



    ros::Rate loop_rate(1);

    /**
     * A count of how many messages we have sent. This is used to create
     * a unique string for each message.
     */
    int count = 0;
    while (ros::ok())
    {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        chatter_pub.publish(msg);

        for(int i = 1; i < 7; i++)
        {
            srv.request.start_time.fromSec(ros::Time::now().sec);
            srv.request.joint_name = "joint_" + std::to_string(i);
            srv.request.effort = 100;
            if (joint_effort_client.call(srv))
            {
                ROS_INFO("Success joint_%d", i);
            }
            else
            {
                ROS_ERROR("Failed to call service joint_%d", i);
            }
        }

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}