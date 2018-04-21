/********************************************************************************
 * gerber_import_client.cpp
 ********************************************************************************/
#include "ros/ros.h"

// include messages
#include "gerber_import/pad.h"
#include "gerber_import/solder_mask.h"

// include server definitions
#include "gerber_import/read_solder_mask.h"
#include "gerber_import/read_solder_maskRequest.h"
#include "gerber_import/read_solder_maskResponse.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gerber_import_client");
  ros::NodeHandle node_handle;
  ros::ServiceClient client = node_handle.serviceClient<gerber_import::read_solder_mask>("gerber_import");

  gerber_import::read_solder_maskRequest req;
  req.filename = "/tmp/test.grb";
  gerber_import::read_solder_maskResponse resp;

  if (client.call(req,resp)) {
    int count = resp.mask.solder_pads.size();
    for (int i=0; i<count; i++) {
      ROS_INFO("x_offset=%f,y_offset=%f,area=%f",
	       resp.mask.solder_pads[i].x_offset,
	       resp.mask.solder_pads[i].y_offset,
	       resp.mask.solder_pads[i].area);
    }
  } else {
    ROS_ERROR("Failed to call service for read_solder_mask");
    return 1;
  }

  return 0;
}
      
