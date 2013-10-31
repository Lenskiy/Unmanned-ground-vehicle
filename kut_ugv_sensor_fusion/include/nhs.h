/*
 * nhs.h
 *
 * This module is to normalise IHLS images.
 *
 * The main function to be called from outside is convert_ihls_to_nhs,
 * which returns a binary image indicating road signs in an image.
 *
 */

#ifndef NHS_H_
#define NHS_H_

#include <cv.h>

using namespace cv;

#define R_CONDITION (h < hue_max || h > hue_min) && s > sat_min
#define B_CONDITION (h < hue_max && h > hue_min) && s > sat_min
#define G_CONDITION (h < hue_max && h > hue_min) && s > sat_min

// Got it from the original Matlab code.
// The values for the red colour.
//#define R_HUE_MAX 11
// increase max and decrease min and sat min = find more color
#define R_HUE_MAX 30
#define R_HUE_MIN 200
//#define R_SAT_MIN 30
#define R_SAT_MIN 5

// The values for the blue colour.
// increase max and decrease min and sat min = find more color
#define B_HUE_MAX 240
#define B_HUE_MIN 80
//#define B_SAT_MIN 40
#define B_SAT_MIN 10

// The values for the green colour.
#define G_HUE_MAX 120
#define G_HUE_MIN 60
#define G_SAT_MIN 40
//#define B_SAT_MIN 20

/**
 * This function receives an IHLS image as an argument, and converts it
 * to an binary image. The pixels that are assumed to be part of a road-sign
 * are going to have value 1, and the rest of picture is going to have value 0.
 *
 * The reason that we have a return matrix and not overriding the original
 * RGB image, is to keep the original RGB values for reference.
 *
 * @param ihls_image
 *   The converted image to IHLS format. It should be in the Mat format of
 *   OpenCv.
 * @param colour
 *   0 = red; 1 = blue; 2 = others; If the value is 2, the rest of parameters
 *   must be provided.
 * @param hue_max
 *   The maximum integer value of hue for thresholding.
 * @param hue_min
 *   The minimum integer value of hue for thresholding.
 * @param sat_min
 *   The minimum integer value of saturation for thresholding.
 *
 * @return
 *   The normalised image in Mat format.
 */
Mat
convert_ihls_to_nhs(Mat ihls_image, int colour = 0, int hue_max = R_HUE_MAX,
    int hue_min = R_HUE_MIN, int sat_min = R_SAT_MIN);

#endif /* NHS_H_ */
