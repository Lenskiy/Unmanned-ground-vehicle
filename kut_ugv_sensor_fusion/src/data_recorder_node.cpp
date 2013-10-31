/*
 * camera_laser_recorder.cpp
 *
 *  Created on: Feb 25, 2013
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

#define WINDOW_NAME "camera"

static laser_geometry::LaserProjection projector_;
static sensor_msgs::PointCloud2 cloud;
static string laser_topic, camera_topic, output_files_names;

static unsigned int file_count = 0;

void callback(const sensor_msgs::LaserScan::ConstPtr& scan, const sensor_msgs::ImageConstPtr& msg)
{
  // Convert from Sick to 2D point cloud
  projector_.projectLaser(*scan, cloud);
  pcl::PointCloud<pcl::PointXYZ> points2d;
  pcl::fromROSMsg(cloud, points2d);

  // Get image and display
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  char key = 0;
  cv::imshow(WINDOW_NAME, cv_ptr->image);
  key = cv::waitKey(3);

  if (key == 's' || key == 'S')
  {
    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(100);

    stringstream ss;
    ss << file_count++;


    cv::imwrite(output_files_names + ss.str() + ".jpg", cv_ptr->image, compression_params);
    pcl::io::savePCDFileASCII(output_files_names + ss.str() + ".pcd", points2d);

    cout << "Saved image: " << output_files_names << ss.str() << ".jpg" << endl;
    cout << "Saved cloud: " << output_files_names << ss.str() << ".pcd" << endl;
  }

}

void help()
{
  ROS_INFO("Usage: camera_laser_recorder\n\t-l: Sick topic\n\t-c: camera topic\n\t-o: output file\n\t Press s to data.\n");
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_laser_recorder");
  ros::NodeHandle nh("~");

  // Getting parameters
  nh.param("laser_topic",laser_topic,string("/scan"));
  nh.param("camera_topic",camera_topic,string("/image_raw"));
  nh.param("output",output_files_names,string("output"));

  // Constructor
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(nh, laser_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, camera_topic, 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::Image> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), laser_sub, image_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  ROS_INFO("WARNING: devices are synchronized, so the speed is setted up to the slowest device!");
  ROS_INFO("Press s to output data\n");

  return 0;
}
