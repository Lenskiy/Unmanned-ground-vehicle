/*
 * bypass_filter.h
 *
 *  Created on: Oct 3, 2013
 *      Author: 0xff
 */

#ifndef BYPASS_FILTER_H_
#define BYPASS_FILTER_H_

#include <algorithm>

#include <laser_geometry/laser_geometry.h>
#include <filters/filter_base.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

#include "point_types.h"

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

namespace kut_ugv_lasers
{

template<typename PointT>
class BypassFilter : public filters::FilterBase<sensor_msgs::PointCloud2>
{
public:
  BypassFilter(){}

  bool configure()
  {
    return true;
  }

  virtual ~BypassFilter()
  {
  }

  bool update(const sensor_msgs::PointCloud2& input_scan, sensor_msgs::PointCloud2& filtered_scan)
  {
    if (&input_scan == &filtered_scan)
    {
      ROS_ERROR("This filter does not currently support in place copying");
      return false;
    }

    typename pcl::PointCloud<PointT>::Ptr in_cloud(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr out_cloud(new pcl::PointCloud<PointT>);

    pcl::fromROSMsg(input_scan, *in_cloud);

    filterPoints(in_cloud, out_cloud);

    filtered_scan.data.clear();
    pcl::toROSMsg(*out_cloud, filtered_scan);
    filtered_scan.header = input_scan.header;

    return true;
  }

  void filterPoints(const typename pcl::PointCloud<PointT>::ConstPtr& cloud_in, typename pcl::PointCloud<PointT>::Ptr& cloud_out)
  {
    pcl::copyPointCloud(*cloud_in, *cloud_out);
  }
};

}

#endif /* BYPASS_FILTER_H_ */
