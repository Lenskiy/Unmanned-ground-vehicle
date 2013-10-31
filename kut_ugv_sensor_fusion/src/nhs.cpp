/*
 * nhs.cpp
 */
#include "nhs.h"
#include <cmath>

Mat
convert_ihls_to_nhs(Mat ihls_image, int colour, int hue_max, int hue_min,
    int sat_min)
{
  if (colour == 2)
    {
      if (hue_max > 255 || hue_max < 0 || hue_min > 255 || hue_min < 0
          || sat_min > 255 || sat_min < 0)
        {
          hue_min = R_HUE_MIN;
          hue_max = R_HUE_MAX;
          sat_min = R_SAT_MIN;
        }
    }
  else if (colour == 1)
    {
      hue_min = B_HUE_MIN;
      hue_max = B_HUE_MAX;
      sat_min = B_SAT_MIN;
    }
  else if (colour == 0)
    {
      hue_min = G_HUE_MIN;
      hue_max = G_HUE_MAX;
      sat_min = G_SAT_MIN;
    }
  else
    {
      hue_min = R_HUE_MIN;
      hue_max = R_HUE_MAX;
      sat_min = R_SAT_MIN;
    }

  assert(ihls_image.channels() == 3);

  Mat nhs_image(ihls_image.rows, ihls_image.cols, CV_8UC1);

  // I put the if before for loops, to make the process faster.
  // Otherwise for each pixel it had to check this condition.
  // Nicer implementation could be to separate these two for loops in
  // two different functions, one for red and one for blue.
  if (colour == 1)
    {
      for (int i = 0; i < ihls_image.rows; ++i)
        {
          const uchar *ihls_data = ihls_image.ptr<uchar> (i);
          uchar *nhs_data = nhs_image.ptr<uchar> (i);
          for (int j = 0; j < ihls_image.cols; ++j)
            {
              uchar s = *ihls_data++;
              // Although l is not being used and we could have
              // replaced the next line with ihls_data++
              // but for the sake of readability, we left it as it it.
              uchar l = *ihls_data++;
              uchar h = *ihls_data++;
              *nhs_data++ = (B_CONDITION) ? 255 : 0;
            }
        }
    }
  else if (colour == 0)
      {
        for (int i = 0; i < ihls_image.rows; ++i)
          {
            const uchar *ihls_data = ihls_image.ptr<uchar> (i);
            uchar *nhs_data = nhs_image.ptr<uchar> (i);
            for (int j = 0; j < ihls_image.cols; ++j)
              {
                uchar s = *ihls_data++;
                // Although l is not being used and we could have
                // replaced the next line with ihls_data++
                // but for the sake of readability, we left it as it it.
                uchar l = *ihls_data++;
                uchar h = *ihls_data++;
                *nhs_data++ = (G_CONDITION) ? 255 : 0;
              }
          }
      }
  else
    {
      for (int i = 0; i < ihls_image.rows; ++i)
        {
          const uchar *ihls_data = ihls_image.ptr<uchar> (i);
          uchar *nhs_data = nhs_image.ptr<uchar> (i);
          for (int j = 0; j < ihls_image.cols; ++j)
            {
              uchar s = *ihls_data++;
              // Although l is not being used and we could have
              // replaced the next line with ihls_data++
              // but for the sake of readability, we left it as it it.
              uchar l = *ihls_data++;
              uchar h = *ihls_data++;
              *nhs_data++ = (R_CONDITION) ? 255 : 0;
            }
        }
    }

  return nhs_image;
}
