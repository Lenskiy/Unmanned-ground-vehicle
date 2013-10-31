//
//  ColorSignDetector.cpp
//  sign_detector
//
//  Created by Artem Lenskiy on 5/31/13.
//  Copyright (c) 2013 Artem Lenskiy. All rights reserved.
//

#include "ColorSignDetector.h"

void makeMat(cv::Mat& colorspart, cv::Mat& mask, cv::Mat& Image)
{
	colorspart = cv::Mat::zeros(Image.size(), CV_8UC3) ;
	mask = cv::Mat::zeros(Image.size(), CV_8UC1) ;
}

float gaussian(float x, float mean, float sigma)
{
	return exp(-((x-mean)/sigma)*((x-mean)/sigma))  ;
}

float sigmoid(float x, float mean, float sigma)
{
	return 1/(1+exp(-(x-mean)/sigma));
}

void constructColorMemb(char mb[][255], float threshold, char color)
{
	int h, s ;
	float val ;
	//precalculate red membership
  	for(h = 0 ; h < 180 ; h++)
  	{
		for(s = 0 ; s < 256 ; s++)
		{
			switch (color)
			{
                case 'r':
                    val = (gaussian(h, 0, 20) + gaussian(h, 180 , 20)) * sigmoid(s, 255 * 0.37, 255 * 0.035);
                    break;
                case 'b':
                    val = gaussian(h, 120, 18) * sigmoid(s, 255 * 0.35, 255 * 0.10);
                    break;
            }
            
			mb[h][s] = threshold < val ? 1 : 0;
		}
  	}
}

void colorFilter(cv::Mat& hsv, cv::Mat& mask, char colormb[][255])
{
	int i, j, k;
	int num_val_in_row = (unsigned)hsv.step ;
	int cur_total_row_vals, cur_total_row_pixs ;
	int total_vals_in_row = (unsigned)hsv.step ;
	unsigned char *data = (unsigned char*)(hsv.data) ;
	unsigned char *cur_color ;
    
	for(i = 0 ; i < mask.rows ; i++)
	{
        cur_total_row_vals = i * total_vals_in_row ;
        cur_total_row_pixs = i * mask.cols ;

       for(j = 0 , k = 0 ; j < num_val_in_row ; j += 3, k++)
        {
			cur_color = data + cur_total_row_vals + j ;
          (colormb[cur_color[0]][cur_color[1]] != 0) ? mask.data[cur_total_row_pixs + k] = 255 : 0;
		}
	}
}

unsigned char blobFinder(cv::Mat& colorChannel, BoundaryBox *bb)
{
    int i;
    float W, H;
    unsigned counter = 0, erosion_size = 1, dilation_size = 3;
    

    CBlobLabeling blob;
	blob.SetParam(colorChannel, 25); // image that labeling, limit mininum pixel for labeling
	blob.DoLabeling(); // do labeling
    
	for(i = 0 ; i < blob.m_nBlobs ; i++)
	{
		// m_nBlobs => number of label
		W = blob.m_recBlobs[i].width;
		H = blob.m_recBlobs[i].height;
		
		if(W != 0 && H != 0)
		{
           if(W / H <= 1.25 && W / H >= 0.75)
            {
                bb[counter].x1 = blob.m_recBlobs[i].x;
                bb[counter].y1 = blob.m_recBlobs[i].y;
                bb[counter].x2 = blob.m_recBlobs[i].x + W;
                bb[counter].y2 = blob.m_recBlobs[i].y + H;
                counter++;
            }
		}
	}
    return counter;
}
