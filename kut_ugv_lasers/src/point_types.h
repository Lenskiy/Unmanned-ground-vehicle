/*
 * point_types.h
 *
 *  Created on: Sep 14, 2013
 *      Author: 0xff
 */

#ifndef POINT_TYPES_H_
#define POINT_TYPES_H_

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
  #include <pcl/sample_consensus/impl/sac_model_circle3d.hpp>
  #include <pcl/search/impl/kdtree.hpp>
#else
  #include <pcl/filters/impl/extract_indices.hpp>
  #include <pcl/filters/impl/statistical_outlier_removal.hpp>
  #include <pcl/segmentation/impl/sac_segmentation.hpp>
  #include <pcl/sample_consensus/impl/prosac.hpp>
  #include <pcl/sample_consensus/impl/lmeds.hpp>
  #include <pcl/sample_consensus/impl/msac.hpp>
  #include <pcl/sample_consensus/impl/mlesac.hpp>
  #include <pcl/sample_consensus/impl/ransac.hpp>
  #include <pcl/sample_consensus/impl/mlesac.hpp>
  #include <pcl/sample_consensus/impl/rmsac.hpp>
  #include <pcl/sample_consensus/impl/rransac.hpp>
  #include <pcl/sample_consensus/impl/sac_model_circle.hpp>
  #include <pcl/sample_consensus/impl/sac_model_plane.hpp>
  #include <pcl/sample_consensus/impl/sac_model_parallel_plane.hpp>
  #include <pcl/sample_consensus/impl/sac_model_perpendicular_plane.hpp>
  #include <pcl/sample_consensus/impl/sac_model_line.hpp>
  #include <pcl/sample_consensus/impl/sac_model_parallel_line.hpp>
  #include <pcl/sample_consensus/impl/sac_model_sphere.hpp>
  #include <pcl/sample_consensus/impl/sac_model_stick.hpp>
  #include <pcl/kdtree/impl/kdtree_flann.hpp>
  #include <pcl/search/impl/organized.hpp>
#endif

namespace kut_ugv_lasers {
  namespace RoadLabelling {
    enum {
      NONE_LABEL = 0,
      ROAD_LABEL = 1,
      EDGE_LABEL = 2,
    };
  }
}

struct EIGEN_ALIGN16 PointXYZILabel
{
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t label;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  inline PointXYZILabel ()
  {
    x = y = z = 0.0f;
    data[3] = 1.0f;
    intensity = 0.0f;
    label = 0;
  }
};

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZILabel,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint32_t, label, label)
)

struct EIGEN_ALIGN16 PointXYZIIndex
{
  PCL_ADD_POINT4D;
  float intensity;
  int32_t index;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  inline PointXYZIIndex ()
  {
    x = y = z = 0.0f;
    data[3] = 1.0f;
    intensity = 0.0f;
    index = 0;
  }
};

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIIndex,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (int32_t, index, index)
)

#endif /* POINT_TYPES_H_ */
