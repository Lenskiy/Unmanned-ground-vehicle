/*
 * moving_object_detector_node.cpp
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

#include <pcl/common/centroid.h>

#include <std_msgs/Header.h>
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

static const float OBJECT_MIN_WIDTH = 0.50;
static const float OBJECT_MIN_HEIGHT = 0.50;
static const float OBJECT_MAX_WIDTH = 3.0;
static const float OBJECT_MAX_HEIGHT = 3.0;

static const float OBJECT_Y_RANGE = 5.0;
static const float OBJECT_MIN_X = 2.0;
static const float OBJECT_MAX_X = 35.0;

class ObjectBuffer
{
  class ObjectDetected
  {
  public:
    ObjectDetected(const ros::Time &time_stamp, unsigned int label)
    {
      time_stamp_ = time_stamp;
      label_ = label;
      pos_detections_ = 0;
      total_detections_ = 0;
    }
    ;
    ros::Time time_stamp_;
    unsigned int label_;
    int pos_detections_;
    int total_detections_;
  };

public:
  ObjectBuffer(int min_detections, float live_time)
  {
    min_detections_ = min_detections;
    live_time_ = live_time;
  }
  ;

  ~ObjectBuffer()
  {
  }
  ;

  void add(const ros::Time &time_stamp, unsigned int label)
  {
    bool object_exist = false;
    std::list<ObjectDetected>::iterator it;
    for (it = object_list_.begin(); it != object_list_.end(); it++)
    {
      (*it).total_detections_++;
      // If object exist
      if ((*it).label_ == label)
      {
        (*it).pos_detections_++;
        (*it).time_stamp_ = time_stamp; // Update time stamp
        object_exist = true;
      }
    }
    // If object does not exist
    if (!object_exist)
    {
      object_list_.push_back(ObjectDetected(time_stamp, label));
    }

    // Clean old objects
    maintain();
  }
  ;

  float getProbability(unsigned int label)
  {
    float ret = 0;

    std::list<ObjectDetected>::iterator it;
    for (it = object_list_.begin(); it != object_list_.end(); it++)
    {
      if ((*it).label_ == label)
      {
        ret = 1 / (1 + exp( -( (float) ( (*it).pos_detections_ - min_detections_ ) ) ));
        break;
      }
    }

    return ret;
  }
  ;

private:
  int min_detections_;
  float live_time_;
  std::list<ObjectDetected> object_list_;

  // Clean the list of old objects
  void maintain()
  {
    std::list<ObjectDetected>::iterator it;
    for (it = object_list_.begin(); it != object_list_.end(); it++)
    {
      float diff = abs( ros::Time::now().toSec() - (*it).time_stamp_.toSec() );
      // Check life
      if (diff > live_time_)
      {
        object_list_.erase(it);
        it--;
      }
    }
  }
  ;
};


class MovingObjectDetector {
public:
	MovingObjectDetector(ros::NodeHandle &nh)
	{
		string laser_object_topic, object_topic;

		nh_ = nh;
		// Getting parameters
		nh.param("laser_object_topic", laser_object_topic, string("/scan/objects"));
		nh.param("threshold", th_, double(5.0));
		nh.param("alpha", alpha_, double(0.8));
		nh.param("display", display_, false);
		nh.param("output", object_topic, string("/crossing_object"));

		pub_ = nh_.advertise<kut_ugv_msgs::WorldObject>(object_topic, 1000);
		sub_ = nh.subscribe(laser_object_topic, 100, &MovingObjectDetector::callBack,
				this);

		ros::spin();
	};


	~MovingObjectDetector()
	{
	};

	void callBack(const kut_ugv_sensor_fusion::lidar_object_listConstPtr& object_list)
	{
	    // Project points
	    pcl::PointCloud<pcl::PointXY> objects;
		findCandidates(object_list, objects);

//		// Check if first time
//		if( prev_2d_scan.empty() )
//		{
//			prev_2d_scan = pcl::PointCloud<pcl::PointXY>(objects);
//			return;
//		}

		// Filter objects based on the position
		filterSpatialArea(objects, objects);
//		cout << "objects size: " << objects.size() << endl;

//		// Compute gradients
//		pcl::PointCloud<pcl::PointXY> gradients;
//		computeGradient(objects, gradients);
//
//		// Compute responses
//		std::vector<float> responses;
//		computeResponses(gradients, responses);
//
//		// Filter responses
//		std::vector<unsigned int> idx;
//		filterResponses(responses, idx);

		// Compute gradient centroid
		if( !objects.empty() )
		{
			pcl::PointXY object_position;
			compute2DCentroid(objects, object_position);
			publish(object_list->header, object_position);
		}

//		// Save to previous
//		prev_2d_scan = pcl::PointCloud<pcl::PointXY>(current_2d_scan);
//
//		if(display_)
//		{
//			CvPlot::clear("Responses");
//			CvPlot::plot("Responses",responses.data(),responses.size(),1,255,0,0);
//		}
	};


private:
	// Ros classes
	ros::NodeHandle nh_;
	ros::Subscriber sub_;
	ros::Publisher pub_;

	// General propose
	pcl::PointCloud<pcl::PointXY> prev_2d_scan;
	double th_; // Threshold
	double alpha_;

	// Visualization
	bool display_;

	// Methods
	// Return a first list of points of objects which has a proper size
	void findCandidates(const kut_ugv_sensor_fusion::lidar_object_listConstPtr& object_list, pcl::PointCloud<pcl::PointXY> &current_2d_scan)
	{
		current_2d_scan.clear();
		for(unsigned int i = 0; i < object_list->object.size(); i++)
		{
			// Check the size of the object
			if( object_list->object[i].width > OBJECT_MIN_WIDTH && object_list->object[i].height > OBJECT_MIN_HEIGHT
					&& object_list->object[i].width < OBJECT_MAX_WIDTH && object_list->object[i].height < OBJECT_MAX_HEIGHT)
			{
				pcl::PointXY p;
				p.x = object_list->object[i].centroid.x;
				p.y = object_list->object[i].centroid.y;
				current_2d_scan.push_back(p);
			}
		}
	}

	void computeGradient(const pcl::PointCloud<pcl::PointXY> &current_2d_scan, pcl::PointCloud<pcl::PointXY> &gradient_scan)
	{
		unsigned int n_points = current_2d_scan.size();
		gradient_scan.clear();
		gradient_scan.resize(n_points);

		for(unsigned int i = 0; i < n_points; i++)
		{
			pcl::PointXY grad;
			grad.x = abs( current_2d_scan[i].x -  prev_2d_scan[i].x ); // dx
			grad.y = abs( current_2d_scan[i].y -  prev_2d_scan[i].y ); // dy
			gradient_scan[i] = grad;
		}
	};

	void computeResponses(const pcl::PointCloud<pcl::PointXY> &gradient_scan, std::vector<float> &responses)
	{
		unsigned int n_points = gradient_scan.size();
		responses.clear();
		responses.resize(n_points);

		for(unsigned int i = 0; i < n_points; i++)
		{
			responses[i] = alpha_ * abs(gradient_scan[i].x) + (1 - alpha_) * abs(gradient_scan[i].y);
		}
	};

	void filterResponses(const std::vector<float> &responses, std::vector<unsigned int> &idx)
	{
		unsigned int n_points = responses.size();
		idx.clear();

		for(unsigned int i = 0; i < n_points; i++)
		{
			if(responses[i] > th_)
			{
				idx.push_back( i );
			}
		}
	};

	void filterSpatialArea(const pcl::PointCloud<pcl::PointXY> &current_2d_scan, pcl::PointCloud<pcl::PointXY> &output)
	{
		pcl::PointCloud<pcl::PointXY> ret;
		for(unsigned int i = 0; i < current_2d_scan.size(); i++)
		{
            if( abs(current_2d_scan[i].y) < OBJECT_Y_RANGE && current_2d_scan[i].x > OBJECT_MIN_X && current_2d_scan[i].x < OBJECT_MAX_X )
            {
            	ret.push_back( current_2d_scan[i] );
            }
		}
		// Return value
		output = ret;
	}

	void filterGradient(const pcl::PointCloud<pcl::PointXY> &gradient_scan, const pcl::PointXY &centroid, std::vector<unsigned int> &idx)
	{
		unsigned int n_points = idx.size();
		std::vector<unsigned int> idx_aux;

		for(unsigned int i = 0; i < n_points; i++)
		{
			float dist = getDistance(centroid, gradient_scan[ idx[i] ]);
//			printf("Grad dist: %f\n", dist);
			if( dist < 1.5 )
			{
				idx_aux.push_back( idx[i] );
			}
		}

		idx = idx_aux;
	}

	void compute2DCentroid(const pcl::PointCloud<pcl::PointXY> &data, pcl::PointXY &centroid)
	{
		centroid.x = 0;
		centroid.y = 0;

		for(unsigned int i = 0; i < data.size(); i++)
		{
			centroid.x += data[i].x;
			centroid.y += data[i].y;
		}

		centroid.x /= data.size();
		centroid.y /= data.size();
	};

	float inline getDistance(const pcl::PointXY &p1, const pcl::PointXY &p2)
	{
	  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
	}

	void publish(const std_msgs::Header &header, const pcl::PointXY &object_position)
	{
	    kut_ugv_msgs::WorldObject publish_me;

	    publish_me.header.frame_id = header.frame_id;
	    publish_me.header.stamp = header.stamp;
	    publish_me.pose.position.x = object_position.x;
	    publish_me.pose.position.y = object_position.y;
	    publish_me.pose.position.z = 0;
		publish_me.pose.orientation.w = 1;
	    publish_me.type = kut_ugv_msgs::WorldObject::OBSTACLE_UNKNOWN;

	    pub_.publish(publish_me);
	};

};

int main(int argc, char** argv) {
	ros::init(argc, argv, "lidar_segmentation");
	ros::NodeHandle nh("~");

	// Parameter server
	MovingObjectDetector moving_object_detector(nh);

	return 0;
}

