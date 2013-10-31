/*
 * laser_intensities_node.cpp
 *
 *  Created on: Aug 29, 2013
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

#include <pcl/common/centroid.h>

#include <kut_ugv_sensor_fusion/lidar_object.h>
#include <kut_ugv_sensor_fusion/lidar_segmentation_labels.h>
#include <kut_ugv_sensor_fusion/lidar_object_list.h>
#include <kut_ugv_msgs/WorldObject.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <vector>

#include <string.h>
#include <iostream>
using namespace std;

// Other functions
#include "cvplot.h"

class LaserIntensities {
public:
	LaserIntensities(ros::NodeHandle &nh)
	{
		string laser_topic;

		nh_ = nh;
		// Getting parameters
		nh.param("laser_topic", laser_topic, string("/roof_laser/scan"));
		nh.param("display", display_, true);

		sub_ = nh.subscribe(laser_topic, 100, &LaserIntensities::callBack,
				this);

		ros::spin();
	};


	~LaserIntensities()
	{
	};


	void callBack(const sensor_msgs::LaserScanConstPtr& scan)
	{
		if(display_)
		{
			CvPlot::clear("Intensities");
			CvPlot::plot("Intensities",scan->intensities.data(),scan->intensities.size(),1,255,0,0);
		}
	};


private:
	// Ros classes
	ros::NodeHandle nh_;
	ros::Subscriber sub_;

	// Visualization
	bool display_;

};

int main(int argc, char** argv) {
	ros::init(argc, argv, "lidar_segmentation");
	ros::NodeHandle nh("~");

	// Parameter server
	LaserIntensities moving_object_detector(nh);

	return 0;
}



