/*
 * ihls.h
 *
 * Header file for IHLS (Improved Hue Luminance Saturation)
 *
 * The main function is convert_rgb_to_ihls which must be called
 * from outside. The rest of functions are provided publicly if
 * someone wants to directly retrieve hue, saturation or luminance
 * based on RGB values.
 *
 */

#ifndef IHLS_H_
#define IHLS_H_

#include <cv.h>

using namespace cv;

/**
 * Returning hue value of RGB.
 */
inline float
retrieve_normalised_hue(unsigned int r, unsigned int g, unsigned int b);

/**
 * Returning luminance value of RGB.
 */
inline float
retrieve_luminance(unsigned int r, unsigned int g, unsigned int b);

/**
 * Returning saturation value of RGB.
 */
inline float
retrieve_saturation(unsigned int r, unsigned int g, unsigned int b);

/**
 * This function converts the an RGB image to IHLS and returns it.
 *
 * The reason that we have a return matrix and not overriding the original
 * RGB image, is to keep the original RGB values for reference.
 *
 */
Mat
convert_rgb_to_ihls(Mat rgb_image);

#endif /* IHLS_H_ */
