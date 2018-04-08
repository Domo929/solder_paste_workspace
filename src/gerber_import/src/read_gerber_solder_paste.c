/*
  Filename: read_gerber_solder_paste.c

  gcc -Wall -g `pkg-config --cflags libgerbv` read_gerber_solder_paste.c `pkg-config --libs libgerbv` -o read_gerber_solder_paste

  To run test
  ./read_gerber_solder_paste test/Top-paste-mask

*/
#include "gerbv.h"
#include <stdlib.h>

void print_aperture_info(gerbv_aperture_t* aperture, gerbv_net_t* net);

int main(int argc, char *argv[])
{
  gerbv_image_t* workingImage;
  gerbv_net_t*   currentNet;

  if (argc < 2) {
    printf("Usage: %s <filename>\n", argv[0]);
    exit(0);
  }

  printf("Loading solder paste file: %s\n", argv[1]);
  
  /* parse and create the image */
  workingImage = gerbv_create_rs274x_image_from_filename(argv[1]);

  /* make sure we parsed the file */
  if (workingImage == NULL) {
    printf("There was an error parsing the file.\n");
    exit(0);
  }

  /* run through all the nets in the layer */
  for (currentNet = workingImage->netlist; currentNet; currentNet = currentNet->next) {
    /* check if the net aperture is a circle and has a diameter < 0.0060 inches */
    if ((currentNet->aperture_state != GERBV_APERTURE_STATE_OFF) &&
	(workingImage->aperture[currentNet->aperture] != NULL)) {

      print_aperture_info(workingImage->aperture[currentNet->aperture],
			  currentNet);
    }
  }

  /* destroy all created structures */
  gerbv_destroy_image(workingImage);

  return 0;
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
