#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <kut_ugv_msgs/WorldObject.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cvaux.hpp>
#include <cv.h>
#include <sensor_msgs/CameraInfo.h>

#define LANE_DETECTOR_HH
//#include "ColorSignDetector.h"
#include "mcv.hh"
#include "InversePerspectiveMapping.hh"
#include "LaneDetector.hh"
#include <kut_ugv_msgs/WorldObject.h>


using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

///Structure to hold lane detector settings


class IPM {
private:
    ros::NodeHandle nh;
    
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    
    ros::Subscriber camera_sub_;
	ros::Publisher stop_line_pub_;
	LaneDetector::CameraInfo cameraInfo;
    LaneDetector::IPMInfo IPM_info;
    kut_ugv_msgs::WorldObject mesg;
    //cv_bridge::CvImage out_msg;
    cv_bridge::CvImagePtr out_msg_;
    
    LaneDetector::LaneDetectorConf lanesConf;
    
    int debug = 0;
    string temp_str;
public:
	string output_topic;
	string stoplane_conf;
	char *stop_lane_conf;
	IPM(ros::NodeHandle &nh) : it_(nh) {

        double temp;
        nh.param("ipmWidth",  IPM_info.width, int(320));
        nh.param("ipmHeight", IPM_info.height, int(240));
        nh.param("ipmLeft",   temp, double(20)); IPM_info.ipmLeft   = (float)temp;
        nh.param("ipmRight",  temp, double(620));IPM_info.ipmRight  = (float)temp;
        nh.param("ipmTop",    temp, double(240));IPM_info.ipmTop    = (float)temp;
        nh.param("ipmBottom", temp, double(480));IPM_info.ipmBottom = (float)temp;
        nh.param("ipmInterpolation", IPM_info.ipmInterpolation, int(0));
        nh.param("display", debug, int(1));
        IPM_info.vpPortion = 0;
        nh.param("output", output_topic, string("/world_object"));
		nh.param("stoplane_conf_url", stoplane_conf, string("/Users/artemlenskiy/ros/ugv_ws/src/kut_ugv/kut_ugv_lane_detector/src/Lanes.conf"));        
        nh.param("cam_info",   temp_str, string("/mono_cam/camera_info"));
 		camera_sub_ = nh.subscribe<sensor_msgs::CameraInfo>(temp_str, 1, &IPM::cameraInfoCb, this);
        nh.param("image",   temp_str, string("/mono_cam/image_rect_color"));
		image_sub_ = it_.subscribe(temp_str, 1, &IPM::imageCb, this);
       // nh.param("ipm",   temp_str, string("/ipm"));
        image_pub_ = it_.advertise("/ipm", 1);
        stop_line_pub_ = nh.advertise<kut_ugv_msgs::WorldObject>(output_topic, 1000);
        
        LaneDetector::mcvInitLaneDetectorConf((char *)stoplane_conf.c_str(), &lanesConf);
	}

	~IPM() {
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
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		//ROS_INFO("CameraImage message received");

		CvMat *mat = cvCreateMat(cv_ptr->image.rows, cv_ptr->image.cols, CV_32FC1);
        CvMat *tchannelImage = cvCreateMat(cv_ptr->image.rows, cv_ptr->image.cols, INT_MAT_TYPE);
        CvMat temp = cv_ptr->image;
		//cvGetMat(&cv_ptr->image, &temp);
		CvMat *raw_mat = cvCloneMat(&temp);
		// convert to single channel
		cvSplit(raw_mat, tchannelImage, NULL, NULL, NULL);
		// convert to float
		cvConvertScale(tchannelImage, mat, 1. / 255);
		cvReleaseMat(&tchannelImage);

		CvSize ipmSize_m = cvSize((int) IPM_info.width, (int) IPM_info.height);
		CvMat *ipm_m = cvCreateMat(ipmSize_m.height, ipmSize_m.width, mat->type);

		list<CvPoint> outPixels_m;
        
		mcvGetIPM(mat, ipm_m, &IPM_info, &cameraInfo, &outPixels_m);
        
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
        
        
        cv::Mat IPM_Mat = cvarrToMat(ipm_m);

        out_msg_.reset(new cv_bridge::CvImage());
        
        out_msg_->encoding = enc::MONO8;
        out_msg_->image = IPM_Mat ;
        out_msg_->header = cv_ptr->header;
        
        image_pub_.publish(out_msg_->toImageMsg());
        
        if(debug){
            imshow("IPM", IPM_Mat);
            cvWaitKey(10);
        }
        
        //IPM_Mat.release();
        cvReleaseMat(&ipm_m);
        cvReleaseMat(&raw_mat);
        cvReleaseMat(&mat);

      }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "IPM");
//  ROS_WARN("stop lane detector startup delayed by 1s");
//  sleep (1);
  ros::NodeHandle nh("~");
  IPM ic(nh);
  ros::spin();
  return 0;
}

