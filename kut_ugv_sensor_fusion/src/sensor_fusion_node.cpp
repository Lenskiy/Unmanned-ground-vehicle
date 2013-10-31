/*
 * camera_lidar_fusion.cpp
 *
 *  Created on: Mar 19, 2013
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

#include <vector>

#include <string.h>
#include <iostream>
using namespace std;

// Other functions
#include "SensorFusion.h"

using namespace sf;


int main(int argc, char** argv)
{
  string laser_topic, camera_topic, lidar_labels_topic_out, calibration_file, svm_model_file;
  cv::Mat K, R, t;
  double max_angle, min_dist, max_dist, grid_size;
  ros::init(argc, argv, "sensor_fusion");
  ros::NodeHandle nh("~");

  // Getting parameters
  nh.param("laser_topic", laser_topic, string("/scan"));
  nh.param("camera_topic", camera_topic, string("/image_raw"));
  nh.param("calibration_url", calibration_file, string("config_file.yaml"));
  nh.param("svm_model_url", svm_model_file, string("svm_model.yaml"));
  nh.param("lidar_max_angle", max_angle, 1.65806281567);
  nh.param("lidar_min_dist", min_dist, 0.0);
  nh.param("lidar_max_dist", max_dist, 81.0);
  nh.param("grid_size", grid_size, 0.4);
//  nh.param("output", output_files_names, string("output"));

  laser_topic = "/bumper_laser/scan";
  camera_topic = "/mono_cam/image_rect_color";
  calibration_file = "/home/dani/catkin_ws/src/kut_ugv_sensor_fusion/calib/calib.yaml";
  svm_model_file = "/home/dani/catkin_ws/src/kut_ugv_sensor_fusion/calib/svm_model.yaml";

  lidar_labels_topic_out = laser_topic + "/labels";

//  SensorFusion sf;
//  sf.load(calibration_file); // Load parameters

  // Param server
  SensorFusion sf(nh);

  // Constructor
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(nh, laser_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, camera_topic, 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::Image> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), laser_sub, image_sub);

  sync.registerCallback(boost::bind(&SensorFusion::callback, &sf, _1, _2));

  ros::spin();

  ROS_INFO("WARNING: devices are synchronized, so the speed is setted up to the slowest device!");
  ROS_INFO("Press s to output data\n");

  return 0;
}

