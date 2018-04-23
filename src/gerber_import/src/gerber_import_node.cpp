/*********************************************************************
 * gerber_import_node.cpp                                            *
 *  Implements server which reads solder_mask information from       *
 *  gerber file in the service request                               *
 *********************************************************************/

#include "ros/ros.h"
#include "gerbv.h"

// include messages
#include "gerber_import/pad.h"
#include "gerber_import/solder_mask.h"

// include server definitions
#include "gerber_import/read_solder_mask.h"
#include "gerber_import/read_solder_maskRequest.h"
#include "gerber_import/read_solder_maskResponse.h"

const int MAX_FILENAME_LENGTH = 256;

bool set_pad_from_aperture_info(gerber_import::pad& pad,
				gerbv_aperture_t* aperture, gerbv_net_t* net);
void print_aperture_info(gerbv_aperture_t* aperture, gerbv_net_t* net);


/**********************************************************************
 * read_solder_mask_file
 * \param req request contains the filename of the solder_mask to read
 * \param resp response returns mask with array of solder pads
 **********************************************************************/
bool read_solder_mask_file(gerber_import::read_solder_maskRequest& req,
			   gerber_import::read_solder_maskResponse& resp)
{
  gerbv_image_t* workingImage;
  gerbv_net_t*   currentNet;

  char filename[MAX_FILENAME_LENGTH];

  // copy file name to non-const char* for old style c routines.
  int size = std::min(MAX_FILENAME_LENGTH, (int)req.filename.length()+1);

  strncpy(filename, req.filename.c_str(), size);
  
  printf("Loading solder paste file: %s\n", filename);

    // parse and create the image
  workingImage = gerbv_create_rs274x_image_from_filename(filename);

  // make sure we parsed the file
  if (workingImage == NULL) {
    printf("There was an error parsing the file.\n");
    return false;
  }

  // run through all the nets in the layer 
  for (currentNet=workingImage->netlist; currentNet; currentNet=currentNet->next) {
    // Check that current net is an aperture
    if ((currentNet->aperture_state != GERBV_APERTURE_STATE_OFF) &&
	(workingImage->aperture[currentNet->aperture] != NULL)) {

      // Planning to a Pose goal
      gerber_import::pad current_pad;

      set_pad_from_aperture_info(current_pad,
				 workingImage->aperture[currentNet->aperture],
				 currentNet);

      // add new pad to vector of solder_mask pads
      resp.mask.solder_pads.push_back(current_pad);

      // debugging printout
      print_aperture_info(workingImage->aperture[currentNet->aperture],
   			  currentNet);
    }
  }

  // destroy all created structures
  gerbv_destroy_image(workingImage);

  return true;
}

/*********************************************************************************
 * main
 * main entry point to gerber import server
 *********************************************************************************/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "gerber_import_server");
  ros::NodeHandle node_handle;

  ros::ServiceServer service = node_handle.advertiseService("gerber_import", read_solder_mask_file);
  ROS_INFO("Reader to server gerber solder mask files.");
  ros::spin();
  
  return 0;
}


/*********************************************************************************
 * set_pad_from_aperture_info 
 * Sets the pad information from the current aperture data
 *********************************************************************************/
bool set_pad_from_aperture_info(gerber_import::pad& pad,
				gerbv_aperture_t* aperture, gerbv_net_t* net)
{
  int valid_offset = 1;
  float area = 0.0;
  float width = 0.0;
  float height = 0.0;
  
  switch(aperture->type) {
  case GERBV_APTYPE_CIRCLE: // a round aperture
    area = M_PI*aperture->parameter[0]/2.0;
    width = aperture->parameter[0];
    height = aperture->parameter[0];
    break;
  case GERBV_APTYPE_RECTANGLE: // a rectangular aperture 
    area = aperture->parameter[0]*aperture->parameter[1];
    width = aperture->parameter[0];
    height = aperture->parameter[1];
    break;
  case GERBV_APTYPE_OVAL: // an ovular (obround) aperture
    area = M_PI*(aperture->parameter[0]/2.0)*(aperture->parameter[1]/2.0);
    width = aperture->parameter[0];
    height = aperture->parameter[1];
    break;
  case GERBV_APTYPE_POLYGON: // a polygon aperture
    area = aperture->parameter[0]*aperture->parameter[1];
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
    // net->stop corresponds to the offset for the center point of the pad
    pad.x_offset = net->stop_x;
    pad.y_offset = net->stop_y;
    // area was calculated based on the pad type above
    pad.area = area;
  }

}

/*********************************************************************************
 * print_aperture_info
 *  Prints the information for the current aperture associated with the net.
 *********************************************************************************/
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
