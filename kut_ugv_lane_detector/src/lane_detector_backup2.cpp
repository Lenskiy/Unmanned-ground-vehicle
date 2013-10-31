#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include "opencv2/objdetect/objdetect.hpp"
#include <cvaux.hpp>
#include <cv.h>
#include <ColorSignDetector.h>
//#ifndef LANE_DETECTOR_HH
#define LANE_DETECTOR_HH

//#include "ColorSignDetector.h"
#include "mcv.hh"
#include "InversePerspectiveMapping.hh"
#include "LaneDetector.hh"
#include <kut_ugv_msgs/WorldObject.h>

using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class StopLineDetector {
private:
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Subscriber camera_sub_;
	image_transport::Publisher image_pub_;
	LaneDetector::CameraInfo cameraInfo;

	kut_ugv_msgs::WorldObject mesg;
	ros::Publisher stop_line_pub_;
    LaneDetector::LaneDetectorConf lanesConf;
    
    cv_bridge::CvImage out_msg;
public:
	string output_topic;
	string stoplane_conf;
	char *stop_lane_conf;
	StopLineDetector(ros::NodeHandle &nh) :
			it_(nh) {
		camera_sub_ = nh.subscribe<sensor_msgs::CameraInfo>("/mono_cam/camera_info", 1, &StopLineDetector::cameraInfoCb, this);
		image_sub_ = it_.subscribe("/ipm", 1, &StopLineDetector::imageCb, this);

		nh.param("stoplane_conf_url", stoplane_conf, string("/Users/artemlenskiy/ros/ugv_ws/src/kut_ugv/kut_ugv_lane_detector/src/Lanes.conf"));
	    nh.param("output", output_topic, string("/world_object"));
	    stop_line_pub_ = nh.advertise<kut_ugv_msgs::WorldObject>(output_topic, 1000);
        LaneDetector::mcvInitLaneDetectorConf(stoplane_conf.c_str(), &lanesConf);
	}

	~StopLineDetector() {

	}

/////////////////////////
	void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& msg) {
		cameraInfo.focalLength = cvPoint2D32f(msg->K.elems[0], msg->K.elems[4]);
		cameraInfo.imageHeight = msg->height;
		cameraInfo.imageWidth = msg->width;
		cameraInfo.pitch = (1.0) * 3.14 / 180;
		cameraInfo.yaw = 0;
		cameraInfo.opticalCenter = cvPoint2D32f(msg->K.elems[2],msg->K.elems[5]);
		cameraInfo.cameraHeight = 1430;
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg) {
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, enc::MONO8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		// convert to mat and get first channel
		//CvMat temp = cv_ptr->image;
		CvMat temp = cv_ptr->image;
		//cvGetMat(&cv_ptr->image, &temp);
		CvMat *ipm_m = cvCloneMat(&temp);
		//cvConvertScale(raw_mat, mat, 1. / 255);
        

	/*	* \param image the input image
		 * \param stopLines a vector of returned stop lines in input image coordinates
		 * \param linescores a vector of line scores returned
		 * \param cameraInfo the camera parameters
		 * \param stopLineConf parameters for stop line detection
		 */
		
        vector<LaneDetector::Line> stopline;
		vector<FLOAT> lineScores;
		LaneDetector::mcvGetStopLines(ipm_m, &stopline, &lineScores, &cameraInfo, &lanesConf);
		double distance;
		char dist[100];
		int line_center;
        cv::Mat IPM(ipm_m);// = cv::Mat::zeros((int)lanesConf.ipmHeight,(int)lanesConf.ipmWidth, CV_8UC1);
//
		for(int i = 0; i < stopline.size(); i++){
		   printf("%f %f %f %f \n", stopline[i].startPoint.x, stopline[i].startPoint.y, stopline[i].endPoint.x, stopline[i].endPoint.y);
           line(IPM, stopline[i].startPoint,stopline[i].endPoint,Scalar(255, 0, 255), 1, 8);
		   line_center = ( stopline[i].startPoint.y+stopline[i].endPoint.y)/2;
		   distance = -0.1183*line_center + 31.333;
		   sprintf(dist, "D = %lf", distance);
           cv::putText(IPM, dist, cv::Point(15, 15), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
		   mesg.header.stamp = msg->header.stamp;
		   mesg.header.frame_id = "/bumper_laser";
		   mesg.pose.position.x = distance;
		   mesg.pose.position.y = 0;
		   mesg.pose.position.z = 0;
		   mesg.pose.orientation.w = 1;
		   mesg.type =kut_ugv_msgs::WorldObject::MARK_STOPLINE;
		   stop_line_pub_.publish(mesg);
		}
         
		imshow("Stop line detection",IPM);
        cvWaitKey(5);
        IPM.release();
        cvReleaseMat(&ipm_m);
        cvReleaseMat(&mat);


      }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stop_line");
//  ROS_WARN("stop lane detector startup delayed by 1s");
//  sleep (1);
  ros::NodeHandle nh("~");
  StopLineDetector ic(nh);
  ros::spin();
  return 0;
}
