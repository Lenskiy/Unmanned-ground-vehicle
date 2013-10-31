/*
 * ImProcessingTools.h
 *
 *  Created on: Aug 27, 2012
 *      Author: Daniel Oñoro Rubio
 *      Email: daniel.onoro@gmail.com
 */

#ifndef IMPROCESSINGTOOLS_H_
#define IMPROCESSINGTOOLS_H_

#include "cv.h"
#include "highgui.h"

using namespace cv;

#include <iostream>

using namespace std;

#include <vector>

#define DEFAULT_PIXEL_WINDOW_NAME "getPixel"


namespace impt {

	/*
		This function is for color segmentation. The input image is segmented and a binary
		image is returned where the white values are all of those which are between
		the color and color + offset values.
	*/
	Mat getThresholdedImage(const Mat &img, Scalar color, Scalar offset);

	//vector <float> calculateBinaryHistogram(Mat imgInput);

	/*
		This function create a frame with the input image and define a mouse
		handler to hold the value of the pixel clicked. To get after the value
		of the pixel clicked call getPixelClicked.
	*/
	void getPixelFromWindow(Mat &img);

	/*
		This function return the value of the last pixel clicked in the function
		getPixelFromWindow.
	*/
	Vec3b getPixelClicked();

	// returns sequence of squares detected on the image.
	// the sequence is stored in the specified memory storage
	void findSquares( const Mat& image, vector<vector<Point> >& squares );

	// the function draws all the squares in the image
	void drawSquares( Mat& image, const vector<vector<Point> >& squares );

	void horizontalEdgesFiler(const Mat &img, Mat &dst);

	void verticalEdgesFiler(const Mat &img, Mat &dst);

};

#endif /* IMPROCESSINGTOOLS_H_ */
