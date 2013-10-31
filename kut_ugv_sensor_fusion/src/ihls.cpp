#include "ihls.h"
#include "math_utils.h"

#include <cmath>

#include <iostream>
using namespace std;

float
retrieve_theta(unsigned int r, unsigned int g, unsigned int b);

/**
 * It calculates theta bases on the equation provided in Valentine thesis.
 *
 * The returned theta is radian.
 */
float
retrieve_theta(unsigned int r, unsigned int g, unsigned int b)
{
  float theta;

  // The numerator part of equation
  float numerator = r - (g * 0.5) - (b * 0.5);

  // The denominator part of equation
  float denominator = (r * r) + (g * g) + (b * b) - (r * g) - (r * b) - (g * b);

  float temp = numerator / sqrtf(denominator);
  theta = acos(temp);

  return theta;
}

/**
 * Calculating the hue value based on the blow formula:
 *
 * H = θ if B <= G
 * H = 2 * pi − θ if B > G
 *
 * The return value is normalised between 0 to 255.
 */
float
retrieve_normalised_hue(unsigned int r, unsigned int g, unsigned int b)
{
  float hue;
  if (b <= g)
    {
      hue = retrieve_theta(r, g, b);
    }
  else
    {
      hue = (2 * M_PI) - retrieve_theta(r, g, b);
    }

  return hue * 255 / (2 * M_PI);
}

/**
 * Luminance is calculated as:
 *
 * L = 0.210R + 0.715G + 0.072B
 */
float
retrieve_luminance(unsigned int r, unsigned int g, unsigned int b)
{
  return (0.210f * r) + (0.715f * g) + (0.072f * b);
}

/**
 * Saturation is calculates as below:
 *
 * S = max(R, G, B) − min(R, G, B)
 */
float
retrieve_saturation(unsigned int r, unsigned int g, unsigned int b)
{
  float saturation;
  unsigned int max = get_maximum(r, g, b);
  unsigned int min = get_minimum(r, g, b);

  saturation = max - min;

  return saturation;
}

Mat
convert_rgb_to_ihls(Mat rgb_image)
{
  assert(rgb_image.channels() == 3);

  Mat ihls_image(rgb_image.rows, rgb_image.cols, CV_8UC3);

  for (int i = 0; i < rgb_image.rows; ++i)
    {
      const uchar* rgb_data = rgb_image.ptr<uchar> (i);
      uchar* ihls_data = ihls_image.ptr<uchar> (i);

      for (int j = 0; j < rgb_image.cols; ++j)
        {
          unsigned int b = *rgb_data++;
          unsigned int g = *rgb_data++;
          unsigned int r = *rgb_data++;
          *ihls_data++ = (uchar) retrieve_saturation(r, g, b);
          *ihls_data++ = (uchar) retrieve_luminance(r, g, b);
          *ihls_data++ = (uchar) retrieve_normalised_hue(r, g, b);
        }
    }

  return ihls_image;
}
