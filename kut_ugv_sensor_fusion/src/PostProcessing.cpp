#include "PostProcessing.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include "opencv2/core/core.hpp"
#include <sstream>
#include <cv.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include "cv.h"
#include "cxcore.h"
#include <vector>
#include <algorithm>
#include <ml.h>

//constructor
PostProcessing::PostProcessing(Mat filename)
{
    image=filename;
}

//destructor
PostProcessing::~PostProcessing()
{

}

//NAME OF FUNCTION: DisplayImage
//PURPOSE:
//    The function will create a window, giving it a name, 
//    and will dislay the image in this window.
//INPUT PARAMETERS:
//     name         type     value/reference               description
//--------------------------------------------------------------------------------
//     name         char*        value               the name of the window
//    image          Mat      value          the image that we want to display 
//
//OUTPUT PARAMETERS:
//     name         type     value/reference               description
//--------------------------------------------------------------------------------
//                             NO OUTPUT
//
void PostProcessing::DisplayImage(const string &name, Mat img)
{
    namedWindow(name,0);
    imshow(name,img);
}

//NAME OF FUNCTION: FilterImage
//PURPOSE:
//    The function will eliminate the noise in the image, using morphological 
//    operations and the median filer. First the source image is dilated,
//    after the contours are filled, the resulted image is eroded and at the 
//    end the image is smooted by the median filter. 
//INPUT PARAMETERS:
//     name         type          value/reference               description
//--------------------------------------------------------------------------------
//                             NO INPUT
//
//OUTPUT PARAMETERS:
//     name         type          value/reference               description
//--------------------------------------------------------------------------------
//      dst          Mat             value                  the filtered image

Mat PostProcessing::FilterImage()
{
    //Morphological operation
    //dilate
    Mat sd=getStructuringElement(MORPH_CROSS, Size(3, 3)); //structuring element used for dilate and erode
    dilate(image, image, sd, cv::Point(0, 0), 2);

    Mat dst=image.clone();                                //destination image; copy of the original image

    // find the contours of the objects
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours(image, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    //draw the filled contours to the destination image
    Scalar color(255,255,255);
    drawContours(dst, contours, -1, color, CV_FILLED, 8);

    //Morphological operations
    //erode
    erode(image, image, sd, cv::Point(0, 0), 2);

    //eliminate nois using a median filter
    for (int i=0;i<5;i++)
        medianBlur(dst,dst,5);

    //output
    return dst;
}

//NAME OF FUNCTION: Elimination
//PURPOSE:
//    The function will eliminate the small objects in the image.
//    First the area of the image is computed. Secondly, for each contour
//    its area is computed and also the the aspect ratio (ration of the 
//    width and the height) of an object. If an object has area less than 
//    1/1500 of image size, and the aspect ratio outside the range of 0.25 and 1.3 then
//    we consider the region as insignificant object, therefore it is eliminated.
//INPUT PARAMETERS:
//     name         type                 value/reference               description
//---------------------------------------------------------------------------------------
//    fimage         Mat                     value          the filtered image, the output 
//                                                           image of the previous method
//   copyCont  vector<vector<Point> >      reference         the contours of the objects
//
//OUTPUT PARAMETERS:
//     name         type          value/reference               description
//--------------------------------------------------------------------------------
//     dest         Mat          value                     the image obtained after 
//                                                      elimination of the small objects
//
Mat PostProcessing::Elimination(Mat fimage, vector<vector<Point> > &copyCont, long int areaRatio, double lowAspectRatio, double highAspectRatio)
{
    long int area=image.size().area();                   //area of the original image
    // cout<<"Aria/1500: "<<area/1500<<endl;

    Mat dst(image.rows, image.cols, CV_8U);   //destination image

    //find the new contours
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours(fimage, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    // cout<<"Dimensiune: "<<contours.size()<<endl;

    //for each contour
    for (unsigned int i = 0; i < contours.size(); i++)
    {
        //associate a rectangle to calculate the width and height
    	 Rect b = boundingRect(contours[i]);// error boundingRect
        printf("ddd\n") ;}

//
//        // compute the aspect ratio
//        double ratio=(double)b.width/(double)b.height;
//
//        //long int areaRegion=b.size.area();
//        long int areaRegion=b.area();
//
//
//        // cout<<"Ratie: "<<ratio<<" Area: "<<areaRegion<<endl;
//
//        if ((areaRegion<area/areaRatio) || ((ratio>highAspectRatio) || (ratio<lowAspectRatio)))  //conditions for eliminating the objects
//        {
//            // cout<<"sters de la pozitia "<<i<<endl;
//        }
//        else
//            copyCont.push_back(contours[i]);
//
//    }
//
//    cout<<copyCont.size()<<endl;
//
//    Scalar color(255,255,255);
//    drawContours(dst, copyCont, -1, color, CV_FILLED, 8);
//
//    //return image after elimination
//    return dst;
//
//	    // find the contours of the objects
//	    vector<vector<Point> > contours;
//	    vector<Vec4i> hierarchy;
//
//	    findContours(fimage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
//
//	    //draw the filled contours to the destination image
	    Scalar color(255,255,255);
	    drawContours(dst, contours, -1, color, CV_FILLED, 8);

	    //Morphological operations
	    //erode
	    //erode(dst,dst,sd);

	    //eliminate nois using a median filter
	    for (int i=0;i<5;i++)
	        medianBlur(dst,dst,5);

	    //output
	    return dst;
}

//NAME OF FUNCTION: Convex
//PURPOSE:
//    The function will recover the shape of the segmented figures .
//    The contours of the shape is found and then the function 'convexHull2'
//    is used. 
//INPUT PARAMETERS:
//     name         type          value/reference              description
//------------------------------------------------------------------------------------
//   eimage         Mat               value            the image after the elimination   
//                                                         of the small shapes 
//   hull     vector<vector<Point>   reference        the memory storage, for each contour
//                                                   the points of  the convex hull will
//                                                             be memorated
// copyCont   vector<vector<Point>   reference          the contours of the objects
//
//
//OUTPUT PARAMETERS:
//     name         type          value/reference               description
//-------------------------------------------------------------------------------------
//     dest         Mat                value           the image obtained after elimination of 
//                                                            the recovery of the shape
//

Mat PostProcessing::Convex(Mat eimage,vector<vector<Point> >&hull,vector<vector<Point> > &copyCont)
{
    Mat dest = Mat::zeros( image.size(), CV_8U ); // the output image
    for (unsigned int i=0;i<copyCont.size();i++)
    {
        convexHull(Mat(copyCont[i]),hull[i], false);    //convex hull operation
    }



    for(unsigned int i = 0; i<copyCont.size(); i++ )   //drawing the points for convex hull
    {
        for (unsigned int j=0;j<hull[i].size();j++)
        {
            Scalar color = Scalar(255, 0, 0);
            //drawContours( drawing, copyCont, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
            //drawContours( dest, hull, i, color, 1, CV_AA, vector<Vec4i>(), 0, Point() );
            circle(dest,hull[i][j],2,color,1,8);
        }
    };

    return dest;

}


//NAME OF FUNCTION: ThresholdedContour
//PURPOSE:
//    The function will recover the shape of the segmented figures .
//    The contours of the shape is found and then the function 'convexHull2'
//    is used.
//INPUT PARAMETERS:
//     name         type          value/reference              description
//------------------------------------------------------------------------------------
//   hull     vector<vector<Point>  reference        the memory storage, for each contour
//                                                   the points of  the convex hull will
//                                                   be memorated
// Contour   vector<vector<Point>   reference        the contours of the objects
//
// extractedCont    vect<vect<pair<int,int>>>        the memory storage, for each contour that
//                                  reference        the points of the function will be stored
//
// dist_threshold   float           value            distance limit between contour points
//                                                   and convex hull edges
//OUTPUT PARAMETERS:
//     name         type            value/reference  description
//-------------------------------------------------------------------------------------
//     dest         Mat             value            the image obtained with the contour
//                                                   close to the edges of convex hull polygon

Mat PostProcessing::ThresholdedContour( vector < vector< Point > > &hull,
                                        vector<vector<Point> > &Contour,
                                        vector<IRO::Contour> &extractedCont,
                                        float dist_threshold )
{
    Mat dest = Mat::zeros( image.size(), CV_8U );

    /*  //DISPLAY ALL CONTOUR POINTS
        cvDrawContours(dest, first_contour,cvScalar(0,255,0),cvScalar(0,255,0),2);//green
    */

    Point hullCurrent, contourPoint, hullNext;

    //Computes the contourPoints
    cout << "hull size =" << hull.size() << endl;
    cout << "contour size =" << Contour.size() << endl;
    for ( unsigned int objectIndex = 0;objectIndex < Contour.size();objectIndex++ )
    {
        int hullIndex = 0;

        hullCurrent = hull[objectIndex][hullIndex];
        contourPoint = Contour[objectIndex][0];

        //searches from the set of hull points which is also the first of the contour points
        while ( hullCurrent.x != contourPoint.x || hullCurrent.y != contourPoint.y )
        {
            hullIndex++;
            hullCurrent = hull[objectIndex][hullIndex];
        }

        //explores the hull contour in inverse order
        cout << endl << hull[0].size() << endl;
        hullIndex = (( hullIndex - 1 ) + hull[objectIndex].size() ) % hull[objectIndex].size();
        hullNext = hull[objectIndex][hullIndex];

        //stores the points of each contour to object vector
        vector< pair <int, int> > object;

        //for each point of the contour
        for ( unsigned int i = 0;i < Contour[objectIndex].size();i++ )
        {
            //explore the contour point
            contourPoint = Contour[objectIndex][i];

            //if the contour point is near to the conex_hull edge add it to the output
            if ( dist_threshold >= distance( hullCurrent, hullNext, contourPoint ) )
            {
                object.push_back( make_pair( contourPoint.x, contourPoint.y ) );
                //cout<<"distance= "<<distance(hullCurrent,hullNext,contourPoint)<<endl;
            }

            //if the explored point is the same than the Hullnext point, then change hullNext and hullcurrent
            if ( hullNext.x == contourPoint.x && hullNext.y == contourPoint.y )
            {
                hullCurrent = hull[objectIndex][hullIndex];
                hullIndex = (( hullIndex - 1 ) + hull[objectIndex].size() ) % hull[objectIndex].size();
                hullNext = hull[objectIndex][hullIndex];
            }
        }

        extractedCont.push_back( object );
    }
    /*
    //DISPLAY THE ORIGINAL CONTOUR
    for(unsigned int i=0;i<Contour.size();i++){
            for(unsigned int j=0;j<Contour[i].size();j++){
                circle(dest,Contour[i][j],1,cvScalar(255,0,0),3 ); //blue
            }
        }
    */
    //display contour points given a threshold
    for ( unsigned int i = 0;i < extractedCont.size();i++ )
    {
        for ( unsigned int j = 0;j < extractedCont[i].size();j++ )
        {
            circle( dest, Point( extractedCont[i][j].first, extractedCont[i][j].second ), 1, cvScalar( 255, 0, 0 ), 3 ); //blue
        }
    }

    return dest;
}

//compute the distance between the edge points (po and pf) , with the current point pc
float PostProcessing:: distance( Point  po, Point pf, Point pc )
{
    float pox = ( float )po.x;
    float poy = ( float )po.y;
    float pfx = ( float )pf.x;
    float pfy = ( float )pf.y;
    float pcx = ( float )pc.x;
    float pcy = ( float )pc.y;

    // In this function, we will compute the altitude of the triangle form by the two points of the convex hull and the one of the contour.
    // It will allow us to remove points far of the convex conserving a degree of freedom

    // Compute the three length of each side of the triangle
    // a will be the base too
    float a = sqrt(pow((float)(pfx - pox), (float)2.00) + pow((float)(pfy - poy), (float)2.00));
    // Compute the two other sides
    float b = sqrt(pow((float)(pcx - pox), (float)2.00) + pow((float)(pcy - poy), (float)2.00));
    float c = sqrt(pow((float)(pfx - pcx), (float)2.00) + pow((float)(pfy - pcy), (float)2.00));

    // Compute S which is the perimeter of the triangle divided by 2
    float s = (a + b + c) / 2.00;

    // Compute the area of the triangle
    float area = sqrt( s*(s - a)*(s - b)*(s-c) );

    // Compute the altitude
    float altitude = 2 * area / a;

    return altitude;

    //return abs(( pfx - pox ) * ( pcy - poy ) - ( pfy - poy ) * ( pcx - pox ) ) / sqrt( pow(( pfx - pox ), 2 ) + pow(( pfy - poy ), 2 ) );
}
