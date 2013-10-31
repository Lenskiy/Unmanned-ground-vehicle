/*
 * statistical_road_filter.h
 *
 *  Created on: Sep 16, 2013
 *      Author: 0xff
 */

#ifndef STATISTICAL_ROAD_FILTER_H_
#define STATISTICAL_ROAD_FILTER_H_

#include <algorithm>

#include <laser_geometry/laser_geometry.h>
#include <filters/filter_base.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

#include "point_types.h"


#define PCL_NO_PRECOMPILE
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace kut_ugv_lasers
{

class StatisticalFilter : public filters::FilterBase<sensor_msgs::PointCloud2>
{
private:
  double road_threshold_;
  double road_edge_threshold_;
  uint32_t mean_k_;
  double stddev_;

public:
  StatisticalFilter() : road_threshold_(0.30), road_edge_threshold_(0.05), mean_k_(10), stddev_(0.5)
  {
  }

  bool configure()
  {
    getParam("road_threshold", road_threshold_);
    getParam("road_edge_threshold", road_edge_threshold_);
    getParam("mean_k", mean_k_);
    getParam("stddev", stddev_);

    return true;
  }

  virtual ~StatisticalFilter()
  {
  }

  bool update(const sensor_msgs::PointCloud2& input_scan, sensor_msgs::PointCloud2& filtered_scan)
  {
    if (&input_scan == &filtered_scan)
    {
      ROS_ERROR("This filter does not currently support in place copying");
      return false;
    }

    pcl::PointCloud<PointXYZIIndex>::Ptr in_cloud(new pcl::PointCloud<PointXYZIIndex>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(input_scan, *in_cloud);

    filterPoints(in_cloud, out_cloud);

    filtered_scan.data.clear();
    pcl::toROSMsg(*out_cloud, filtered_scan);
    filtered_scan.header = input_scan.header;

    return true;
  }

  void filterPoints(const pcl::PointCloud<PointXYZIIndex>::ConstPtr& cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out)
  {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr road_inliers(new pcl::PointIndices());
    pcl::PointIndices::Ptr road_inliers_inv(new pcl::PointIndices());
    pcl::PointIndices::Ptr road_edge_inliers(new pcl::PointIndices());
    pcl::PointCloud<PointXYZIIndex>::Ptr tmp_cloud(new pcl::PointCloud<PointXYZIIndex>);

    pcl::StatisticalOutlierRemoval<PointXYZIIndex> sor;

    // Create the segmentation object
    pcl::SACSegmentation<PointXYZIIndex> seg_road;
    // Optional
    seg_road.setOptimizeCoefficients(true);
    // Mandatory
    seg_road.setModelType(pcl::SACMODEL_LINE);
    seg_road.setMethodType(pcl::SAC_RANSAC);
    seg_road.setMaxIterations(1000);
    seg_road.setDistanceThreshold(road_threshold_);

    // Create the filtering object
    pcl::ExtractIndices<PointXYZIIndex> extract(true);

    // Segment the largest linear component from the cloud
    seg_road.setInputCloud(cloud_in);
    seg_road.segment(*road_inliers, *coefficients);
    if (road_inliers->indices.size() == 0)
    {
      ROS_WARN("Could not estimate a linear model for the given dataset");
      return;
    }

    // Keep everything except road
    extract.setInputCloud(cloud_in);
    extract.setIndices(road_inliers);
    extract.setNegative(true);
    extract.filter(road_inliers_inv->indices);

    // Statistical outlier removal
    sor.setInputCloud(cloud_in);
    sor.setIndices(road_inliers);
    sor.setMeanK(mean_k_);
    sor.setStddevMulThresh(stddev_);
    sor.setNegative(true);
    sor.filter(road_edge_inliers->indices);

    // Concatenate RANSAC filtered points with statistically removed ones
    road_edge_inliers->indices.insert(road_edge_inliers->indices.end(),
                                      road_inliers_inv->indices.begin(),
                                      road_inliers_inv->indices.end());
    // Extract
    extract.setInputCloud(cloud_in);
    extract.setIndices(road_edge_inliers);
    extract.setNegative(false);
    extract.filter(*tmp_cloud);
    pcl::copyPointCloud(*tmp_cloud, *cloud_out);
  }
};

}

#endif /* STATISTICAL_ROAD_FILTER_H_ */
