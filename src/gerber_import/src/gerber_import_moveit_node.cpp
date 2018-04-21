/*********************************************************************
 * gerber_import_node.cpp                                            *
 *********************************************************************/

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include "gerbv.h"

#include "gerber_import/pad.h"
#include "gerber_import/solder_mask.h"

char filename[] = "/tmp/solder_paste_mask.grb";

bool set_pose_from_aperture_info(geometry_msgs::Pose& target_pose,
				 gerbv_aperture_t* aperture, gerbv_net_t* net);
void print_aperture_info(gerbv_aperture_t* aperture, gerbv_net_t* net);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gerber_import");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "arm";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("odom_combined");
  visual_tools.deleteAllMarkers();

  visual_tools.loadRemoteControl();
  
  visual_tools.trigger();

  // Getting Basic Information
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());


  gerbv_image_t* workingImage;
  gerbv_net_t*   currentNet;

  printf("Loading solder paste file: %s\n", filename);
  
  // parse and create the image
  workingImage = gerbv_create_rs274x_image_from_filename(filename);

  // make sure we parsed the file
  if (workingImage == NULL) {
    printf("There was an error parsing the file.\n");
    exit(0);
  }

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // run through all the nets in the layer 
  for (currentNet=workingImage->netlist; currentNet; currentNet=currentNet->next) {
    // Check that current net is an aperture
    if ((currentNet->aperture_state != GERBV_APERTURE_STATE_OFF) &&
	(workingImage->aperture[currentNet->aperture] != NULL)) {

      // Planning to a Pose goal
      geometry_msgs::Pose target_pose;

      set_pose_from_aperture_info(target_pose,
				  workingImage->aperture[currentNet->aperture],
				  currentNet);

      move_group.setPoseTarget(target_pose);

      bool success =
	(move_group.plan(my_plan) ==
	 moveit::planning_interface::MoveItErrorCode::SUCCESS);

      ROS_INFO_NAMED("gerber_import",
		     "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

      // Visualizing plans
      // ^^^^^^^^^^^^^^^^^
      // We can also visualize the plan as a line with markers in Rviz.
      ROS_INFO_NAMED("gerber_import", "Visualizing plan 1 as trajectory line");
      visual_tools.publishAxisLabeled(target_pose, "pose");
      visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
      visual_tools.trigger();
      visual_tools.prompt("next step");

      // Moving to a pose goal
      move_group.move();

      // debugging printout
      print_aperture_info(workingImage->aperture[currentNet->aperture],
   			  currentNet);
    }
  }

  // destroy all created structures
  gerbv_destroy_image(workingImage);

  // clean up before exit
  ros::shutdown();
  
  return 0;
}


bool set_pose_from_aperture_info(geometry_msgs::Pose& target_pose,
				 gerbv_aperture_t* aperture, gerbv_net_t* net)
{
  int valid_offset = 1;
  
  switch(aperture->type) {
  case GERBV_APTYPE_CIRCLE: /*!< a round aperture */
    printf("type: CIRCLE, diameter: %f, num: %d, offset: ",
	   aperture->parameter[0],
	   aperture->nuf_parameters);
    break;
  case GERBV_APTYPE_RECTANGLE: /*!< a rectangular aperture */
    printf("type: RECTANGLE, width: %f, height: %f, num: %d, offset: ",
	   aperture->parameter[0],
	   aperture->parameter[1],
	   aperture->nuf_parameters);
    break;
  case GERBV_APTYPE_OVAL: /*!< an ovular (obround) aperture */
    printf("type: OVAL, width: %f, height: %f, num: %d, offset: ",
	   aperture->parameter[0],
	   aperture->parameter[1],
	   aperture->nuf_parameters);
    break;
  case GERBV_APTYPE_POLYGON: /*!< a polygon aperture */
    printf("type: POLYGON, width: %f, height: %f, num: %d, offset: ",
	   aperture->parameter[0],
	   aperture->parameter[1],
	   aperture->nuf_parameters);
    break;
  case GERBV_APTYPE_NONE:
    // skip non apertures...
    break;
  default:
    printf("ERROR: unsupported aperture.\n");
    valid_offset = 0;
    break;
  }

  if (valid_offset != 0) {
    target_pose.orientation.w = 1.0;
    target_pose.position.x = (net->stop_x + net->start_x)/2.0;
    target_pose.position.y = (net->stop_y + net->start_y)/2.0;;
    target_pose.position.z = 0.0;
  }

}

void print_aperture_info(gerbv_aperture_t* aperture, gerbv_net_t* net)
{
  int print_offset = 1;
  
  switch(aperture->type) {
  case GERBV_APTYPE_CIRCLE: /*!< a round aperture */
    printf("type: CIRCLE, diameter: %f, num: %d, offset: ",
	   aperture->parameter[0],
	   aperture->nuf_parameters);
    break;
  case GERBV_APTYPE_RECTANGLE: /*!< a rectangular aperture */
    printf("type: RECTANGLE, width: %f, height: %f, num: %d, offset: ",
	   aperture->parameter[0],
	   aperture->parameter[1],
	   aperture->nuf_parameters);
    break;
  case GERBV_APTYPE_OVAL: /*!< an ovular (obround) aperture */
    printf("type: OVAL, width: %f, height: %f, num: %d, offset: ",
	   aperture->parameter[0],
	   aperture->parameter[1],
	   aperture->nuf_parameters);
    break;
  case GERBV_APTYPE_POLYGON: /*!< a polygon aperture */
    printf("type: POLYGON, width: %f, height: %f, num: %d, offset: ",
	   aperture->parameter[0],
	   aperture->parameter[1],
	   aperture->nuf_parameters);
    break;
  case GERBV_APTYPE_NONE:
    // skip non apertures...
    break;
  default:
    printf("ERROR: unsupported aperture.\n");
    print_offset = 0;
    break;
  }

  if (print_offset != 0) {
    printf("start { x: %f, y: %f }, stop { x: %f, y: %f }\n",
	   net->start_x, net->start_y,
	   net->stop_x, net->stop_y);
  }

}
