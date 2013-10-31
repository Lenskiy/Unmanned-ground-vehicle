//
//  ColorSignDetector.h
//  sign_detector
//
//  Created by Artem Lenskiy on 5/31/13.
//  Copyright (c) 2013 Artem Lenskiy. All rights reserved.
//

#ifndef sign_detector_ColorSignDetector_h
#define sign_detector_ColorSignDetector_h
//#include "BlobLabeling.h"
#include "opencv2/imgproc/imgproc.hpp"

struct BoundaryBox{
    unsigned x1, y1, x2, y2;
};

float gaussian(float hue, float sat, float sigma) ;
float sigmoid(float hue, float sat, float sigma) ;

void constructColorMemb(char mb[][255], float threshold, char color);

void colorFilter(cv::Mat& hsv, cv::Mat& colorfiltered, char greenmb[][255]) ;
//unsigned char blobFinder(cv::Mat& mask, BoundaryBox *bb);



#endif
