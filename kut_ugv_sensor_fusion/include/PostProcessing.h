# include "cv.h"
# include "cxcore.h"
# include "highgui.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include "RotationalOffset.h"

using namespace std;
using namespace cv;

class PostProcessing
{
    Mat image;
public:
    //Constructor and destructors
    PostProcessing(Mat);
    ~PostProcessing();

    //Methods
    void DisplayImage(const string &, Mat);
    Mat FilterImage();
    Mat Elimination(Mat,vector<vector<Point> >&,long int, double, double);
    Mat Convex(Mat, vector<vector<Point> >&hull,vector<vector<Point> > &copyCont);
    Mat ThresholdedContour(vector<vector<Point> >&hull,vector<vector<Point> > &Contour, vector<IRO::Contour> &extractedCont, float );

private:
    float distance(Point  po, Point pf, Point pc);    
};
