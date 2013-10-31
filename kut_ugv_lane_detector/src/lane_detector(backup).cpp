#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <pcl_ros/point_cloud.h>


// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

//#ifndef LANE_DETECTOR_HH
#define LANE_DETECTOR_HH

#include "mcv.hh"
#include "InversePerspectiveMapping.hh"
#include "LaneDetector.hh"




namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";


class ImageConverter
{
    

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber camera_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher lines_could_pub_;
    LaneDetector::CameraInfo cameraInfo;
    
    typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;


public:
  ImageConverter() :
      it_(nh_)
  {
    camera_sub_ = nh_.subscribe<sensor_msgs::CameraInfo>("/mono_cam/camera_info", 1, &ImageConverter::cameraInfoCb, this);
    image_sub_ = it_.subscribe("/mono_cam/image_raw", 1, &ImageConverter::imageCb, this);
      
    //image_pub_ = it_.advertise("out", 1);
    

    
   // lines_could_pub_ = nh_.advertise<PCLCloud> ("/lines_cloud", 100);
    
      //cv::namedWindow(WINDOW);
  }

  ~ImageConverter(){
   // cv::destroyWindow(WINDOW);
  }

    void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& msg){
        cameraInfo.focalLength = cvPoint2D32f(msg->K.elems[0], msg->K.elems[4]);
        cameraInfo.imageHeight = msg->height;
        cameraInfo.imageWidth = msg->width;
        cameraInfo.pitch  =   (1.0)*3.14/180;
        cameraInfo.yaw = 0 ;
        cameraInfo.opticalCenter = cvPoint2D32f(msg->K.elems[2], msg->K.elems[5]);
        cameraInfo.cameraHeight = 1430;
        //camera_sub_.shutdown();
    }
    

  void imageCb(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

       //ROS_INFO("CameraImage message received");
    //cv::Mat canny;

    //cv::Canny(cv_ptr->image, canny, 50, 200);

    //cv::imshow(WINDOW, cv_ptr->image);
    //cv::imshow("Canny", canny);

    //cv::waitKey(3);

      CvMat *raw_mat;
      CvMat *mat;
      
      // convert to mat and get first channel
      CvMat temp = cv_ptr->image;
      //cvGetMat(&cv_ptr->image, &temp);
      raw_mat = cvCloneMat(&temp);
      // convert to single channel
      CvMat* tchannelImage = cvCreateMat(cv_ptr->image.rows, cv_ptr->image.cols, INT_MAT_TYPE);
      cvSplit(raw_mat, tchannelImage, NULL, NULL, NULL);
      // convert to float
      mat = cvCreateMat(cv_ptr->image.rows, cv_ptr->image.cols, FLOAT_MAT_TYPE);
      cvConvertScale(tchannelImage, mat, 1./255);
      // destroy
      cvReleaseMat(&tchannelImage);

      
      // detect lanes
      vector<FLOAT> lineScores, splineScores;
      vector<LaneDetector::Line> lanes;
      vector<LaneDetector::Spline> splines;
       LaneDetector::IPMInfo ipmInfo;
      
      
      LaneDetector::LaneDetectorConf lanesConf;
      mcvInitLaneDetectorConf("/home/seungwoo/catkin_ws/src/kut_ugv/kut_ugv_lane_detector/src/Lanes.conf", &lanesConf);
      
      //LaneDetector::CameraInfo cameraInfo;
      //mcvInitCameraInfo("/Users/artemlenskiy/ros/ugv_ws/src/lane_detector/src/CameraInfo.conf", &cameraInfo);
      
      //printf("dims = (%f, %f), ext = (%f, %f), center = (%f, %f), focal = (%f, %f)\n", cameraInfo.imageWidth, cameraInfo.imageHeight, cameraInfo.pitch, cameraInfo.yaw, cameraInfo.opticalCenter.x, cameraInfo.opticalCenter.y, cameraInfo.focalLength.x, cameraInfo.focalLength.y);
      CvSize ipmSize = cvSize((int)lanesConf.ipmWidth, (int)lanesConf.ipmHeight);

      CvMat *ipm = cvCreateMat(ipmSize.height, ipmSize.width, mat->type);
      mcvGetIPM(mat, ipm, &ipmInfo, &cameraInfo);
      list<CvPoint> outPixels;
      list<CvPoint>::iterator outPixelsi;
      
      mcvGetIPM(mat, ipm, &ipmInfo, &cameraInfo, &outPixels);

      LaneDetector::LineState state;
      mcvGetLanes(mat, raw_mat, &lanes, &lineScores, &splines, &splineScores, &cameraInfo, &lanesConf, &state);
      //image_pub_.publish(cv_ptr->toImageMsg());
      CvMat *dbIpmImage = cvCreateMat(ipm->height, ipm->width, ipm->type);
      vector<LaneDetector::Spline> dbIpmSplines = state.ipmSplines;
      
          for (int i=0; i<(int)dbIpmSplines.size(); i++)
              mcvDrawSpline(dbIpmImage, dbIpmSplines[i], CV_RGB(0,0,0), 1);
      
   //   LaneDetector::SHOW_IMAGE(dbIpmImage, "Lanes IPM with lines", 10);
      
  /*    PCLCloud laneCloud;
      for (int i=0; i<(int)dbIpmSplines.size(); i++){
          CvMat *pixels = mcvGetBezierSplinePixels(dbIpmSplines[i], .05, cvSize(ipm->width, ipm->height), false);
      
          //if no pixels
          if (!pixels)
              continue;
      
          //draw pixels in image with that color
          for (int i=0; i<pixels->height-1; i++){
              pcl::PointXYZ toPush;
              toPush.x = (int)cvGetReal2D(pixels, i, 0);
              toPush.y = (int)cvGetReal2D(pixels, i, 1);
              toPush.z = 0;
 
              laneCloud.points.push_back(toPush);
          }
      }
      
      lines_could_pub_.publish(laneCloud.makeShared());*/


          
      
      // show detected lanes
      CvMat *imDisplay = cvCloneMat(raw_mat);
      // convert to BGR
      //     cvCvtColor(raw_mat, imDisplay, CV_RGB2BGR);
      if (lanesConf.ransacLine && !lanesConf.ransacSpline)
          for(int i=0; i<lanes.size(); i++)
              mcvDrawLine(imDisplay, lanes[i], CV_RGB(0,125,0), 3);
      // print lanes
      if (lanesConf.ransacSpline)
      {
          for(int i=0; i<splines.size(); i++)
          {
              if (splines[i].color == LaneDetector::LINE_COLOR_YELLOW)
                  mcvDrawSpline(imDisplay, splines[i], CV_RGB(255,255,0), 3);
              else
                  mcvDrawSpline(imDisplay, splines[i], CV_RGB(0,255,0), 3);
              // print numbers?
              if (true)
              {
                  char str[256];
                  sprintf(str, "%d", i);
                  LaneDetector::mcvDrawText(imDisplay, str,
                                            cvPointFrom32f(splines[i].points[splines[i].degree]), 1, CV_RGB(0, 0, 255));
              }
          }
      }

     LaneDetector::SHOW_IMAGE(imDisplay, "Detected Lanes", 1);
     //////////////////
     cvReleaseMat(&raw_mat);
     cvReleaseMat(&mat);
     cvReleaseMat(&imDisplay);
     cvReleaseMat(&ipm);
    //cv::imshow("Detected Lanes!", imDisplay);
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
