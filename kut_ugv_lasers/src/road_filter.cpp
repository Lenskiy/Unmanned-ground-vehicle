/*
 * road_filter.cpp
 *
 *  Created on: Aug 26, 2013
 *      Author: 0xff
 */

#include <sensor_msgs/PointCloud2.h>
#include <pluginlib/class_list_macros.h>

#include "statistical_road_filter.h"
#include "bypass_filter.h"

PLUGINLIB_EXPORT_CLASS(kut_ugv_lasers::StatisticalFilter, filters::FilterBase<sensor_msgs::PointCloud2>)
PLUGINLIB_EXPORT_CLASS(kut_ugv_lasers::BypassFilter<pcl::PointXYZ>, filters::FilterBase<sensor_msgs::PointCloud2>)
