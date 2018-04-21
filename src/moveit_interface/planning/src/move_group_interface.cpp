/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include "math.h"

geometry_msgs::Pose::_orientation_type euler_angle2quaternion(float y, float p, float r)
{
    tf::Matrix3x3 orientaion_mat;
    tf::Quaternion q;
    geometry_msgs::Pose::_orientation_type q_pos;

    orientaion_mat.setEulerYPR(y, p, r);
    orientaion_mat.getRotation(q);

    q_pos.w = q.getW();
    q_pos.x = q.getX();
    q_pos.y = q.getY();
    q_pos.z = q.getZ();

    return q_pos;
}

float degree2rad(float deg)
{
    return deg * M_PI / 180.0;
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "manipulator";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in Rviz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("odom_combined");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in Rviz
  visual_tools.loadRemoteControl();

  // Rviz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 0.5; // above head of PR2
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to Rviz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose1;
    target_pose1.orientation = euler_angle2quaternion(degree2rad(90), 0, degree2rad(180));
    ROS_INFO("orientation1: %f, %f, %f, %f", target_pose1.orientation.w, target_pose1.orientation.x, target_pose1.orientation.y, target_pose1.orientation.z);

  target_pose1.position.x = 0;
  target_pose1.position.y = 0.4;
  target_pose1.position.z = 0.4;
  move_group.setPoseTarget(target_pose1);
    move_group.setStartStateToCurrentState();

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in Rviz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // Moving to a pose goal is similar to the step above
  // except we now use the move() function. Note that
  // the pose goal we had set earlier is still active
  // and so the robot will try to move to that goal. We will
  // not use that function in this tutorial since it is
  // a blocking function and requires a controller to be active
  // and report success on execution of a trajectory.

  /* Uncomment below line when working with a real robot */
    //move_group.execute(my_plan);
    move_group.move();
    ros::WallDuration sleep_time(5.0);
    sleep_time.sleep();
    // move_group.move();


    // Planning to a Pose goal
    // ^^^^^^^^^^^^^^^^^^^^^^^
    // We can plan a motion for this group to a desired pose for the
    // end-effector.
    geometry_msgs::Pose target_pose2;
    target_pose2.orientation = euler_angle2quaternion(degree2rad(90), 0, degree2rad(180));

    target_pose2.position.x = 0;
    target_pose2.position.y = 0.2;
    target_pose2.position.z = 0.5;
    move_group.setPoseTarget(target_pose2);
    move_group.setStartStateToCurrentState();
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (pose goal) %s", success ? "" : "FAILED");
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose1, "pose1");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("next step");

    //move_group.execute(my_plan);
    move_group.move();
    sleep_time.sleep();


    geometry_msgs::Pose target_pose3;
    target_pose3.orientation = euler_angle2quaternion(degree2rad(90), 0, degree2rad(180));

    target_pose3.position.x = 0;
    target_pose3.position.y = 0.2;
    target_pose3.position.z = 0.5005;
    move_group.setPoseTarget(target_pose3);
    move_group.setStartStateToCurrentState();
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (pose goal) %s", success ? "" : "FAILED");
    ROS_INFO_NAMED("tutorial", "Visualizing plan 3 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose1, "pose1");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("next step");

    //move_group.execute(my_plan);
    move_group.move();
    sleep_time.sleep();





    ros::shutdown();
    return 0;
}
