/*
 * pedestrian_detector_node.cpp
 *
 *  Created on: Aug 15, 2013
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

#include <kut_ugv_msgs/WorldObject.h>
#include <kut_ugv_sensor_fusion/lidar_object.h>
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

using namespace sf;

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

class PedestrianDetection
{
public:
  PedestrianDetection(ros::NodeHandle &nh)
  {
    string camera_lidar_calibration_file, output_topic, svm_model_file;
    double grid_size;

    nh_ = nh;
    // Getting parameters
    nh_.param("calibration_url", camera_lidar_calibration_file, string("config_file.yaml"));
    nh_.param("svm_model_url", svm_model_file, string("svm_model.yaml"));
    nh_.param("output", output_topic, string("/pedestrian_detector"));
    nh_.param("display", display_, false);
    nh_.param("record", record_, false);
    nh_.param("video_output", video_path_, string("camera.avi"));

//    camera_lidar_calibration_file = "/home/dani/catkin_ws/src/kut_ugv/kut_ugv_sensor_fusion/calib/calib.yaml";
//    svm_model_file = "/home/dani/catkin_ws/src/kut_ugv/kut_ugv_sensor_fusion/calib/svm_model.yaml";

    // Initialize camera-lidar converter
    lidar_to_image_ = new LidarImageConverter(camera_lidar_calibration_file);

    // Set Classifier
    classifier_ = new Classifier(svm_model_file);

    // Set Detection buffer
    detection_buffer = new ObjectBuffer(15, 0.5);

    // Publisher
    pub_ = nh_.advertise<kut_ugv_msgs::WorldObject>(output_topic, 1000);
  }
  ;

  ~PedestrianDetection()
  {
    delete lidar_to_image_;
    delete detection_buffer;
  }
  ;

  void callBack(const sensor_msgs::LaserScan::ConstPtr& scan, const sensor_msgs::ImageConstPtr& msg,
                const kut_ugv_sensor_fusion::lidar_object_listConstPtr& object_list)
  {
    // Project points
    pcl::PointCloud<pcl::PointXY> points2d;
    sensor_msgs::PointCloud2 cloud;
    laser_geometry::LaserProjection projector;
    // Convert from Sick to 2D point cloud
    projector.projectLaser(*scan, cloud);
    pcl::fromROSMsg(cloud, points2d);
    // Convert from lidar to camera points
    std::vector<cv::Point2f> camera_points;
    fromPCL2Camera(points2d, camera_points);

    // Get object list
    std::vector<LidarObject> objectList;
    convert2LidarObject(objectList, object_list);

    // Find Bounding Boxes
    std::vector<cv::Rect> bbList;
    bbList = getBoundingBoxes(objectList, camera_points);

    // Get Image
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

    // Get image size
    cv::Size imSize = cv_ptr->image.size();

    // Filter BB out of the image
    std::vector<unsigned int> bb_inside_image_idx;
    for (unsigned int i = 0; i < bbList.size(); i++)
    {
      cv::Rect r = bbList[i];

      if (!((r.x < 0) || ((r.x + r.width) > imSize.width) || (r.y < 0) || ((r.y + r.height) > imSize.height)
          || (r.width == 0) || (r.height == 0)))
      {
        // Delete all BB that are out of the image
        bb_inside_image_idx.push_back(i);
      }
    }

    // Classify images
    std::vector<unsigned int> pedestrian_idx;
    for (unsigned int i = 0; i < bb_inside_image_idx.size(); i++)
    {
      cv::Mat roi(cv_ptr->image, bbList[bb_inside_image_idx[i]]);

      float response = classifier_->predict(roi);

      if (response == 1)
      {
        pedestrian_idx.push_back(bb_inside_image_idx[i]);
        pedestrian_label.push_back(objectList[bb_inside_image_idx[i]].label);

        detection_buffer->add(ros::Time::now(), objectList[bb_inside_image_idx[i]].label);
      }
    }

    if (display_)
    {
      // Plot lidar points
      for (unsigned int i = 0; i < camera_points.size(); i++)
      {
        cv::circle(cv_ptr->image, camera_points[i], 3, cv::Scalar(100, 200, 0), -1);
      }
    }

    // Track pedestrians
    for (unsigned int i = 0; i < pedestrian_label.size(); i++)
    {
      bool is_label_matching = false;
      unsigned int match_idx;
      for (unsigned int j = 0; j < objectList.size(); j++)
      {
        if (pedestrian_label[i] == objectList[j].label)
        {
          if (objectList[j].centroid.x != 0 || objectList[j].centroid.y != 0)
          {
            if( detection_buffer->getProbability(pedestrian_label[i]) > 0.5 )
            {
              is_label_matching = true;
              match_idx = j;
              break;
            }
          }
        }
      }

      if (is_label_matching)
      {

        if (display_)
        {
          cv::rectangle(cv_ptr->image, bbList[match_idx], cv::Scalar(255, 100, 0), 2);
          cv::putText(cv_ptr->image, "Pedestrian", cv::Point(bbList[match_idx].x, bbList[match_idx].y - 5),
                      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 100, 0), 2);
        }

        // Publish data
        publishPedestrian(objectList[match_idx], scan);
      }
      else
      {
        pedestrian_label.erase(pedestrian_label.begin() + i);
        i--;
      }
    }

    if (display_)
    {
//      // Plot pedestrian text
//      for (unsigned int i = 0; i < pedestrian_idx.size(); i++)
//      {
//        cv::rectangle(cv_ptr->image, bbList[pedestrian_idx[i]], cv::Scalar(0, 200, 0), 2);
//
//        cv::putText(cv_ptr->image, "Pedestrian",
//                    cv::Point(bbList[pedestrian_idx[i]].x, bbList[pedestrian_idx[i]].y - 5), cv::FONT_HERSHEY_SIMPLEX,
//                    0.5, cv::Scalar(0, 200, 0), 2);
//      }
      cv::imshow(WINDOW_CAMERA_NAME, cv_ptr->image);
      cv::waitKey(3);

      if (!vw_camera.isOpened())
      {
        vw_camera.open(video_path_, CV_FOURCC('x', 'v', 'i', 'd'), FPS, cv_ptr->image.size(), true);

        if (!vw_camera.isOpened())
        {
          std::cout << "Can't create video writer. Nothing will be recorded" << std::endl;
        }
      }
      else
      {
        // Save frame
        vw_camera << cv_ptr->image;
      }
    }

  }
  ;

private:
  // Ros classes
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  // Lidar to image converter
  LidarImageConverter *lidar_to_image_;

  // Classifier
  Classifier *classifier_;
  std::vector<unsigned int> pedestrian_label;

  // Detection buffer
  ObjectBuffer *detection_buffer;

  // Visualizer
  bool display_;
  // Recorder
  bool record_;
  cv::VideoWriter vw_camera;
  std::string video_path_;

  // Methods
  void convert2LidarObject(std::vector<LidarObject> &objectList,
                           const kut_ugv_sensor_fusion::lidar_object_listConstPtr& object_list)
  {
    for (unsigned int i = 0; i < object_list->object.size(); i++)
    {
      kut_ugv_sensor_fusion::lidar_object obj_aux = object_list->object[i];
      LidarObject obj_converted;

      obj_converted.centroid.x = obj_aux.centroid.x;
      obj_converted.centroid.y = obj_aux.centroid.y;
      obj_converted.min_x = obj_aux.min_x;
      obj_converted.max_x = obj_aux.max_x;
      obj_converted.min_y = obj_aux.min_y;
      obj_converted.max_y = obj_aux.min_y;
      obj_converted.min_x_idx = obj_aux.min_x_idx;
      obj_converted.max_x_idx = obj_aux.max_x_idx;
      obj_converted.min_y_idx = obj_aux.min_y_idx;
      obj_converted.max_y_idx = obj_aux.max_y_idx;
      obj_converted.width = obj_aux.width;
      obj_converted.height = obj_aux.height;
      obj_converted.label = obj_aux.label;
      for (unsigned int j = 0; j < obj_aux.idxs.size(); j++)
      {
        obj_converted.idxs.push_back(obj_aux.idxs[j]);
      }

      objectList.push_back(obj_converted);
    }
  }
  ;

  void publishPedestrian(const LidarObject &object, const sensor_msgs::LaserScanConstPtr& scan)
  {
    kut_ugv_msgs::WorldObject publish_me;

    publish_me.header.frame_id = scan->header.frame_id;
    publish_me.header.stamp = scan->header.stamp;
    publish_me.pose.position.x = object.centroid.x;
    publish_me.pose.position.y = object.centroid.y;
    publish_me.pose.position.z = 0;
	publish_me.pose.orientation.w = 1;
    publish_me.type = kut_ugv_msgs::WorldObject::OBSTACLE_PEDESTRIAN;
    publish_me.id = object.label;

    pub_.publish(publish_me);
  }
  ;

  void fromPCL2Camera(const pcl::PointCloud<pcl::PointXY> &points2d, std::vector<cv::Point2f> &camera_points)
  {
    double data_point[3];
    data_point[1] = 0;

    for (size_t i = 0; i < points2d.size(); i++)
    {
      // Set 3D point from lidar
      data_point[0] = -points2d[i].y;
      data_point[2] = points2d[i].x;

      cv::Mat Pl(3, 1, CV_64F, data_point);
      // Convert 3D point to 2D image point

      cv::Point2f p = lidar_to_image_->from3d2image(Pl);

      // Save 2D image point
      camera_points.push_back(p);
    }
  }
  ;

  std::vector<cv::Rect> getBoundingBoxes(const std::vector<LidarObject> &objectList,
                                         const std::vector<cv::Point2f> &camera_points)
  {
    std::vector<cv::Rect> bbList;
    bbList.resize(objectList.size());

    for (int i = 0; i < objectList.size(); i++)
    {
      std::vector<int> point_idx = objectList[i].idxs;
      int min_idx, max_idx;
      float min_x, max_x;

      min_x = camera_points[point_idx[0]].x;
      min_idx = point_idx[0];
      max_x = camera_points[point_idx[0]].x;
      max_idx = point_idx[0];

      for (int j = 1; j < point_idx.size(); j++)
      {
        if (min_x > camera_points[point_idx[j]].x)
        {
          min_x = camera_points[point_idx[j]].x;
          min_idx = point_idx[j];
        }
        if (max_x < camera_points[point_idx[j]].x)
        {
          max_x = camera_points[point_idx[j]].x;
          max_idx = point_idx[j];
          //printf("2\n");
        }
      }

      cv::Rect r;

      // Filter objects that are too big or too small
      if (!((objectList[i].width > 1)
          || (objectList[i].width < 0.3) && (objectList[i].height > 1) && (objectList[i].height < 0.3)))
      {

        // 2D point
        float x_width = camera_points[max_idx].x - camera_points[min_idx].x;
        cv::Point2f tli_point(camera_points[min_idx].x, camera_points[min_idx].y);
        cv::Point2f bri_point(camera_points[max_idx].x, camera_points[max_idx].y);

        r = cv::Rect(tli_point, bri_point);

        // Increase BB size
        int x_inc = r.width * 0.25 + 10; // Increments

        r.x -= x_inc;
        r.width += 2 * x_inc;
        r.height += 2 * r.width;
        r.y -= r.height * 0.55;

      }

      bbList[i] = r;
    }

    return bbList;
  }

};

int main(int argc, char** argv)
{
  string laser_topic, laser_labels_topic, camera_topic, camera_lidar_calibration_file, output_topic, svm_model_file;
  ros::init(argc, argv, "pedestrian_detection");
  ros::NodeHandle nh("~");

  nh.param("laser_topic", laser_topic, string("/scan"));
  nh.param("laser_labels_topic", laser_labels_topic, laser_topic + "/objects");
  nh.param("camera_topic", camera_topic, string("/image_raw"));

  laser_topic = "/bumper_laser/scan";
  laser_labels_topic = "/bumper_laser/scan/objects";
  camera_topic = "/mono_cam/image_rect_color";

  // Param server
  PedestrianDetection pedestrian_detection(nh);

  // Subscribers
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(nh, laser_topic, 1);
  message_filters::Subscriber<kut_ugv_sensor_fusion::lidar_object_list> lidar_labels_sub(nh, laser_labels_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, camera_topic, 1);

  // Subscribe and synchronize
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::Image,
      kut_ugv_sensor_fusion::lidar_object_list> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), laser_sub, image_sub, lidar_labels_sub);

  sync.registerCallback(boost::bind(&PedestrianDetection::callBack, &pedestrian_detection, _1, _2, _3));

  ROS_INFO("WARNING: devices are synchronized, so the speed is setted up to the slowest device!");

  ros::spin();

  return 0;
}

