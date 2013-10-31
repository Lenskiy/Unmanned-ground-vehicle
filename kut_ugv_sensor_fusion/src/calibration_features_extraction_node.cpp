/*
 * calibration_features_extraction_node.cpp
 *
 *  Created on: May 27, 2013
 *      Author: Daniel Oñoro Rubio
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/ros/conversions.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>
using namespace std;

// Other functions
#include "SensorFusion.h"

using namespace sf;


void help()
{
  ROS_INFO("Usage: camera_laser_recorder\n\t-l: Sick topic\n\t-c: camera topic\n\t-o: output file\n\t Press s to data.\n");
}


int main(int argc, char** argv)
{
  string laser_topic, camera_topic, config_file, output_files_names;
  ros::init(argc, argv, "callibration_feature_tool");
  ros::NodeHandle nh("~");

  // Getting parameters
  nh.param("laser_topic",laser_topic,string("/scan"));
  nh.param("camera_topic",camera_topic,string("/image_raw"));
  nh.param("config", config_file, string("config_file.yaml"));
  nh.param("output",output_files_names,string("output"));

//  laser_topic = "/lms511/scan";
//  camera_topic = "/camera_firewire/camera/image_raw";
//  config_file = "/home/dani/ros_autonomous_vehicle/sandbox/sensor_fusion/config/config.yaml";

  CalibrationFeaturesTool calibTool;
  calibTool.setUp(config_file, output_files_names); // Load parameters

  printf("Press 'P' to stop the image.\n");

  // Constructor
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(nh, laser_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, camera_topic, 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::Image> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), laser_sub, image_sub);

  sync.registerCallback(boost::bind(&CalibrationFeaturesTool::callback, &calibTool, _1, _2));

  ros::spin();

  ROS_INFO("WARNING: devices are synchronized, so the speed is setted up to the slowest device!");
  ROS_INFO("Press s to output data\n");

  return 0;
}
