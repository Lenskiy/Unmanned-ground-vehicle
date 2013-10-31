//
//  ColorSignDetector.cpp
//  sign_detector
//
//  Created by Artem Lenskiy on 5/31/13.
//  Copyright (c) 2013 Artem Lenskiy. All rights reserved.
//

#include "ColorSignDetector.h"

float gaussian(float x, float mean, float sigma){
	return exp(-((x-mean)/sigma)*((x-mean)/sigma))  ;
}

float sigmoid(float x, float mean, float sigma){
	return 1/(1+exp(-(x-mean)/sigma));
}


void constructColorMemb(char mb[][255], float threshold, char color){
	int h, s ;
	float val ;
	//precalculate red membership
  	for(h = 0 ; h < 180 ; h++)
		for(s = 0 ; s < 256 ; s++)
		{
			switch (color) {
                case 'r':
                    val = (gaussian(h, 0, 15) + gaussian(h, 180 , 15)) * sigmoid(s, 255 * 0.24, 255 * 0.15);
                    break;
                case 'g':
                    val = gaussian(h, 65, 15) * sigmoid(s, 255 * 0.4, 255 * 0.05);
                    break;
                case 'b':
                    val = gaussian(h, 120, 15) * sigmoid(s, 255 * 0.5, 255 * 0.05);
                    break;
                case 'y':
                	 	val = gaussian(h, 30, 20) * sigmoid(s, 255 * 0.35, 255 * 0.37);
                	  break;
            }
            
			mb[h][s] = threshold < val ? 1 : 0;
		}
}


void colorFilter(cv::Mat& hsv, cv::Mat& mask, char colorMemb[][255])
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
            (colorMemb[cur_color[0]][cur_color[1]] != 0) ? mask.data[cur_total_row_pixs + k] = 255 : 0;
		}
	}
}

