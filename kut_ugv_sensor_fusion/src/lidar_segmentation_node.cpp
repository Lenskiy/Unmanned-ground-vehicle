/*
 * lidar_segmentation_node.cpp
 *
 *  Created on: August 15, 2013
 *      Author: Daniel Oñoro Rubio
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/ros/conversions.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <kut_ugv_sensor_fusion/lidar_object.h>
#include <kut_ugv_sensor_fusion/lidar_segmentation_labels.h>
#include <kut_ugv_sensor_fusion/lidar_object_list.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <vector>

#include <string.h>
#include <iostream>
using namespace std;

// Other functions
#include "SensorFusion.h"

#define DISPLAY_WIDTH 1024

using namespace sf;

class LidarSegmentationNode {
public:
	LidarSegmentationNode(ros::NodeHandle &nh) {
		string laser_topic, lidar_labels_topic_out, lidar_object_topic_output;

		nh_ = nh;
		// Getting parameters
		nh.param("laser_topic", laser_topic, string("/scan"));
		nh.param("grid_size", grid_size_, 0.4);
		nh.param("display", display_, false);
		nh.param("labels_output", lidar_labels_topic_out,
				laser_topic + "/labels");
		nh.param("object_output", lidar_object_topic_output,
				laser_topic + "/objects");

		laser_segmentation_ = NULL;
		laser_display_ = NULL;

		object_pub_ = nh_.advertise<kut_ugv_sensor_fusion::lidar_object_list>(
				lidar_object_topic_output, 1000);
		labels_pub_ = nh_.advertise<
				kut_ugv_sensor_fusion::lidar_segmentation_labels>(
				lidar_labels_topic_out, 1000);
		sub_ = nh.subscribe(laser_topic, 100, &LidarSegmentationNode::callBack,
				this);

		ros::spin();
	}
	;

	~LidarSegmentationNode() {
		delete laser_segmentation_;
	}
	;

	void callBack(const sensor_msgs::LaserScanConstPtr& scan) {
		if (!laser_segmentation_) {
			double max_angle, min_dist, max_dist;
			max_angle = scan->angle_max;
			min_dist = scan->angle_min;
			max_dist = scan->range_max;
			laser_segmentation_ = new LaserSegmentation(max_angle, min_dist,
					max_dist, grid_size_);

			if (display_) {
				laser_display_ = new LaserDisplay(max_angle, min_dist, max_dist,
						grid_size_);
			}
		}

		// Project points
		pcl::PointCloud<pcl::PointXY> points2d;
		sensor_msgs::PointCloud2 cloud;
		laser_geometry::LaserProjection projector;
		// Convert from Sick to 2D point cloud
		projector.projectLaser(*scan, cloud);
		pcl::fromROSMsg(cloud, points2d);

		// Segment lidar data
		std::vector<int> labels;
		unsigned int n_segments = 0;
		laser_segmentation_->process(points2d, labels, n_segments);

		// Track labels
		std::vector<LidarObject> objectList;
		std::vector<cv::Rect> bbList;

		objectList = object_tracker_.initObjectList(points2d, labels,
				n_segments);
		object_tracker_.track(objectList, labels, 1.5);

		// Publish data
		publishData(labels, objectList, scan);

		if (display_) {
			cv::Mat lidar_map = laser_display_->getMap(points2d, labels);
			cv::flip(lidar_map, lidar_map, -1);
			cv::Size im_map_size = lidar_map.size();
			cv::resize(lidar_map, lidar_map,
					cv::Size(DISPLAY_WIDTH,
							im_map_size.height * DISPLAY_WIDTH
									/ im_map_size.width), cv::INTER_LANCZOS4);
			cv::imshow(WINDOW_LASER_NAME, lidar_map);
			cv::waitKey(1);
		}

	}
	;

private:
	double grid_size_;

	// Ros classes
	ros::NodeHandle nh_;
	ros::Subscriber sub_;
	ros::Publisher labels_pub_, object_pub_;

	// Segmentation Class
	LaserSegmentation *laser_segmentation_;
	LidarObstacleTracker object_tracker_;

	// Visualizer
	// Visualization variables
	LaserDisplay *laser_display_;
	bool display_;

	// Labels publisher
	void publishData(const std::vector<int> &labels,
			const std::vector<LidarObject> &objectList,
			const sensor_msgs::LaserScanConstPtr& scan) {
		kut_ugv_sensor_fusion::lidar_segmentation_labels labels_pub;
		kut_ugv_sensor_fusion::lidar_object_list objects_pub;

		// Publish Labels
		labels_pub.header.frame_id = scan->header.frame_id;
		labels_pub.header.stamp = scan->header.stamp;
		labels_pub.labels.resize(labels.size());
		memcpy(labels_pub.labels.data(), labels.data(),
				labels.size() * sizeof(int));
		labels_pub_.publish(labels_pub);

		// Publish Objects
		objects_pub.header.frame_id = scan->header.frame_id;
		objects_pub.header.stamp = scan->header.stamp;
		for (unsigned int i = 0; i < objectList.size(); i++) {
			kut_ugv_sensor_fusion::lidar_object object;
			for (unsigned int j = 0; j < objectList[i].idxs.size(); j++) {
				object.idxs.push_back(objectList[i].idxs[j]);
			}
			geometry_msgs::Point p;
			p.x = objectList[i].centroid.x;
			p.y = objectList[i].centroid.y;
			object.centroid = p;
			object.min_x = objectList[i].min_x;
			object.max_x = objectList[i].max_x;
			object.min_y = objectList[i].min_y;
			object.max_y = objectList[i].max_y;
			object.min_x_idx = objectList[i].min_x_idx;
			object.max_x_idx = objectList[i].max_x_idx;
			object.min_y_idx = objectList[i].min_y_idx;
			object.max_y_idx = objectList[i].max_y_idx;
			object.width = objectList[i].width;
			object.height = objectList[i].height;
			object.label = objectList[i].label;

			objects_pub.object.push_back(object);
		}
		object_pub_.publish(objects_pub);
	}
	;

};

int main(int argc, char** argv) {
	string laser_topic, lidar_labels_topic_out;
	double max_angle, min_dist, max_dist, grid_size;
	ros::init(argc, argv, "lidar_segmentation");
	ros::NodeHandle nh("~");

	// Param server
	LidarSegmentationNode lidar_segmentation(nh);

	// Constructor
	message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(nh,
			laser_topic, 1);

	return 0;
}

