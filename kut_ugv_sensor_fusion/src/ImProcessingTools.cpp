/*
 * ImProcessingTools.cpp
 *
 *  Created on: Aug 27, 2012
 *      Author: Daniel Oñoro Rubio
 *      Email: daniel.onoro@gmail.com
 */

#include "ImProcessingTools.h"

namespace impt {

static int thresh = 50, N = 11;

// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
static double angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
void findSquares( const Mat& image, vector<vector<Point> >& squares )
{
    squares.clear();

    Mat pyr, timg, gray0(image.size(), CV_8U), gray;

    // down-scale and upscale the image to filter out the noise
    pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
    pyrUp(pyr, timg, image.size());
    vector<vector<Point> > contours;

    // find squares in every color plane of the image
    for( int c = 0; c < 3; c++ )
    {
        int ch[] = {c, 0};
        mixChannels(&timg, 1, &gray0, 1, ch, 1);

        // try several threshold levels
        for( int l = 0; l < N; l++ )
        {
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if( l == 0 )
            {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                Canny(gray0, gray, 0, thresh, 5);
                // dilate canny output to remove potential
                // holes between edge segments
                dilate(gray, gray, Mat(), Point(-1,-1));
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l+1)*255/N;
            }

            // find contours and store them all as a list
            findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

            vector<Point> approx;

            // test each contour
            for( size_t i = 0; i < contours.size(); i++ )
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if( approx.size() == 4 &&
                    fabs(contourArea(Mat(approx))) > 1000 &&
                    isContourConvex(Mat(approx)) )
                {
                    double maxCosine = 0;

                    for( int j = 2; j < 5; j++ )
                    {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if( maxCosine < 0.3 )
                        squares.push_back(approx);
                }
            }
        }
    }
}


// the function draws all the squares in the image
void drawSquares( Mat& image, const vector<vector<Point> >& squares )
{
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];
        int n = (int)squares[i].size();
        polylines(image, &p, &n, 1, true, Scalar(0,255,0), 3, CV_AA);
    }
}


void horizontalEdgesFiler(const Mat &img, Mat &dst)
{
	// Enhance edges
	Mat kernel_right(3,1,CV_8SC1);
	kernel_right.data[0] = -1;
	kernel_right.data[1] = 0;
	kernel_right.data[2] = 1;

	Mat kernel_left(3,1,CV_8SC1);
	kernel_left.data[0] = 1;
	kernel_left.data[1] = 0;
	kernel_left.data[2] = -1;

	Mat left_edges;
	filter2D(img, left_edges, CV_8U, kernel_left);

	Mat right_edges;
	filter2D(img, right_edges, CV_8U, kernel_right);

	Mat both_edges = left_edges + right_edges;

	/*
	 * Find edges in half size. This is useful for blurred images
	 */
	Mat half_size;
	resize(img, half_size, Size(), 0.5, 0.5);

	Mat right_edges_half;
	filter2D(half_size, right_edges_half, CV_8U, kernel_right);

	Mat left_edges_half;
	filter2D(half_size, left_edges_half, CV_8U, kernel_left);

	Mat both_edges_half = left_edges_half + right_edges_half;

	// Rescale to full size again
	resize(both_edges_half, both_edges_half, img.size());

	dst = (both_edges_half + both_edges);

	threshold(dst, dst, 60, 255, THRESH_BINARY);
}

void verticalEdgesFiler(const Mat &img, Mat &dst)
{
	// Enhance edges
	Mat kernel_right(1,3,CV_8SC1);
	kernel_right.data[0] = -1;
	kernel_right.data[1] = 0;
	kernel_right.data[2] = 1;

	Mat kernel_left(3,1,CV_8SC1);
	kernel_left.data[0] = 1;
	kernel_left.data[1] = 0;
	kernel_left.data[2] = -1;

	Mat left_edges;
	filter2D(img, left_edges, CV_8U, kernel_left);

	Mat right_edges;
	filter2D(img, right_edges, CV_8U, kernel_right);

	Mat both_edges = left_edges + right_edges;

	/*
	 * Find edges in half size. This is useful for blurred images
	 */
	Mat half_size;
	resize(img, half_size, Size(), 0.5, 0.5);

	Mat right_edges_half;
	filter2D(half_size, right_edges_half, CV_8U, kernel_right);

	Mat left_edges_half;
	filter2D(half_size, left_edges_half, CV_8U, kernel_left);

	Mat both_edges_half = left_edges_half + right_edges_half;

	// Rescale to full size again
	resize(both_edges_half, both_edges_half, img.size());

	dst = (both_edges_half + both_edges);

	threshold(dst, dst, 60, 255, THRESH_BINARY);
}

Mat getThresholdedImage(const Mat &img, Scalar color, Scalar offset) {

	Mat imgThreshed;

	// Control offsets
	if ((color[1] + offset[1]) > 255)
		offset[1] = 255 - color[1];

	if ((color[2] + offset[2]) > 255)
		offset[2] = 255 - color[2];

	if (color[0] + offset[0] > 179) {
		/*
		 In OpenCV hue goes from 0-179, so if the value is bigger than 179 is
		 necessary to make it in two steps and to combine them.
		 */

		Mat firstSep;
		inRange(img, color,
				Scalar(179, (color[1] + offset[1]), (color[2] + offset[2])),
				firstSep);

		Mat secondStep;

		inRange(img, Scalar(0, color[1], color[2]),
				Scalar(((color[0] + offset[0]) - 180), (color[1] + offset[1]),
						(color[2] + offset[2])), secondStep);

//		imshow("step1",firstSep);
//		imshow("secondStep",secondStep);

//		add(firstSep,secondStep,imgThreshed);

		imgThreshed = secondStep | firstSep;

//		imshow("sum",imgThreshed);

	} else {
		inRange(img, color,
				Scalar((color[0] + offset[0]), (color[1] + offset[1]),
						(color[2] + offset[2])), imgThreshed);
	}

	return imgThreshed;
}

// Define global variable needed for the handler
static Vec3b _pixel;

void mouseHandler(int event, int x, int y, int flags, void* param) {

	if (event == EVENT_LBUTTONDOWN) {

		Mat *img = (Mat*) param; // Cast matrix

		// Create the string to display
		char label[20];
		sprintf(label, "(%d, %d, %d)", (int) img->at<Vec3b>(y, x)[0],
				(int) img->at<Vec3b>(y, x)[1], (int) img->at<Vec3b>(y, x)[2]);

		// Plot pixel value in std output
		cout << "Plot pixel[" << x << "," << y << "]: " << label << endl;

		// Draw a rectangle to make a background for the text
		rectangle(*img, Point(5, 0), Point(265, 35), Scalar(0, 0, 0), -1, 8, 0);

		putText(*img, label, Point(5, 25), FONT_HERSHEY_SIMPLEX, 1.,
				Scalar(255, 100, 0), 2);

		// redraw image
		imshow(DEFAULT_PIXEL_WINDOW_NAME, *img);

		// Set pixel variable
		_pixel = img->at<Vec3b>(y, x);
	}
}

void getPixelFromWindow(Mat &img) {
	if (img.empty()) {
		cout << "Error: image null in function getPixelFromWindowMat." << endl;
		return;
	}

	/* create a window and install mouse handler */
	namedWindow(DEFAULT_PIXEL_WINDOW_NAME);

	setMouseCallback(DEFAULT_PIXEL_WINDOW_NAME, mouseHandler, &img);

	// Draw image
	imshow(DEFAULT_PIXEL_WINDOW_NAME, img);

	cvWaitKey(0);

	destroyWindow(DEFAULT_PIXEL_WINDOW_NAME);
}

Vec3b getPixelClicked() {
	return _pixel;
}

}
