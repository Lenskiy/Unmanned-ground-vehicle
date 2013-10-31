/*
 * SensorFusion.h
 *
 *  Created on: Mar 25, 2013
 *      Author: Daniel Oñoro Rubio
 */

#ifndef SENSORFUSION_H_
#define SENSORFUSION_H_

#include <ros/ros.h>
#include <ros/publisher.h>
#include <kut_ugv_sensor_fusion/lidar_segmentation_labels.h>
#include <kut_ugv_msgs/WorldObject.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>

#include <boost/foreach.hpp>

#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/ml/ml.hpp>

//#include <pcl/visualization/cloud_viewer.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <vector>
//#include <list>
#include "DList.h"
#include <string.h>
#include <iostream>
#include <fstream>

#define MAX_COLOURS 100

#define PI 3.1415926

//#define RECORD_CAMERA
#define FPS 25.0

// Laser display scale
#define DISP_SCALE 1.5

namespace sf
{

/******************************************************************************/
/*                            LidarImageConverter                             */
/******************************************************************************/
class LidarImageConverter
{
public:
  LidarImageConverter(const cv::Mat& _K, const cv::Mat& _R, const cv::Mat& _t);
  LidarImageConverter(std::string calibration_file);


  virtual ~LidarImageConverter();

  /*
   * Convert a 3D point (X Y Z)' from 3d to image coordinates.
   */
  cv::Point2f from3d2image(const cv::Mat& Pl);

  /*
   * Convert a 3D point (X Y Z)' from 3d to image coordinates.
   */
  cv::Point2f from3d2image(const cv::Point3f& Pw);

private:
  cv::Mat K;
  cv::Mat R;
  cv::Mat t;


  /*
   * loadCalibrationParams data from a yaml file.
   */
  bool loadCalibrationParams(std::string file);
};

struct grid_bin
{
  DList<size_t> points_list;
  unsigned int label;
  bool isEmpty,hasLabel;
  Node<cv::Point>* ptr_bin_position_list;
  grid_bin()
  {
    label = 0;
    isEmpty = true;
    hasLabel = false;
  }
};


/******************************************************************************/
/*                             LaserSegmentation                              */
/******************************************************************************/
class LaserSegmentation
{
public:
  LaserSegmentation(double _max_angle, double _min_dist, double _max_dist, double _grid_size);
  virtual ~LaserSegmentation();

  /*
   * This function gets the list of 2D points and segment them into connected
   * regions.
   *    -Input:
   *            -points2d: 2D points data
   *    -Output:
   *            -labels: list of labels for each point.
   *            -n_segments: total number of segments.
   */
  void process(const pcl::PointCloud<pcl::PointXY> &points2d, std::vector<int> &labels, unsigned int &n_segments);

  /*
   * This function gets the list of 2D points and segment according the
   * difference between points.
   *    -Input:
   *            -scan: lidar raw data
   *            -d_th: distance dependence threshold
   *            -bias: distance dependence bias
   *    -Output:
   *            -labels: list of labels for each point.
   *            -n_segments: total number of segments.
   */
  void diff_segmentation(const sensor_msgs::LaserScan& scan, std::vector<int> &labels, int &n_segments, float d_th, float bias);

private:
  // Internal data struct
  grid_bin** map;
  DList<cv::Point> bin_position_list;
  DList<grid_bin*> ocupated_bins_list;

  // General variables
  int map_width, map_height;
  unsigned int current_label;
  pcl::PointXY origin;
  double bin_size;

  // Methods

  /*
   * This function returns a list of the neighborhood of the element
   * bin_pos in a 8-connected grid.
   */
  void get_neighborhood(cv::Point bin_pos, DList<cv::Point>& neighborhood);

  /*
   * Get and element and label it
   */
  void label_element(cv::Point bin_pos);

  /*
   * Calculates distance dependence between two consecutive lidar beams.
   */
  float distance_dependence(float delta_theta, float r0, float r1);
};

/******************************************************************************/
/*                          CalibrationFeaturesTool                           */
/******************************************************************************/

struct display_bin
{
  std::vector<pcl::PointXY> points_list;
  bool isEmpty;

  display_bin()
  {
    isEmpty = true;
  }
};

class LaserDisplay
{
public:
  LaserDisplay(double _max_angle, double _min_dist, double _max_dist, double _grid_size);
  virtual ~LaserDisplay();

  /*
   * This function gets the list of 2D points and segment them into connected
   * regions.
   *    -Input:
   *            -points2d: 2D points data
   *    -Output:
   *            -cv::Mat: image with a representation of the laser data.
   */
  cv::Mat getMap(const pcl::PointCloud<pcl::PointXY> &points2d, const std::vector<int> &labels);

  /*
   * This function gets a rectangular region of interest, and returns all the
   * points contained by that region.
   *    -Input:
   *            -cv::Rect: Roi area
   *    -Output:
   *            -cv::Mat: matrix with the laser data.
   */
  cv::Mat getRoiPoints(cv::Rect roi_area);

  /*
   * This function get an index and returns color correspounding to that label.
   */
  cv::Scalar getLabelColor(int label_idx);

private:
  // Internal data struct
  display_bin** map;
  DList<cv::Point> bin_position_list;
  DList<display_bin*> ocupated_bins_list;

  // General variables
  int map_width, map_height;
  pcl::PointXY origin;
  double bin_size;

  // Visualization variables
  cv::Mat img_map;
  cv::Vec3b white_color;
  std::vector<cv::Vec3b> colour_vector_;
};

// Define some constants
#define WINDOW_CAMERA_NAME "Camera"
#define WINDOW_CAMERA_LINES_NAME "Camera Lines"
#define WINDOW_LASER_NAME "Laser"
#define WINDOW_LASER_LINES_NAME "Laser Lines"
#define ROI_WIDTH 50
#define ROI_HEIGHT 100
#define ROI_LASER 25
#define N_LINES 3

class CalibrationFeaturesTool
{
public:
  CalibrationFeaturesTool();
  ~CalibrationFeaturesTool();

  /*
   * loadCalibrationParams data from a yaml file.
   */
  bool setUp(std::string input_file,std::string _output_file);

  void callback(const sensor_msgs::LaserScan::ConstPtr& scan, const sensor_msgs::ImageConstPtr& msg);

  void mouseCameraHandler(int event, int x, int y, int flags, const cv::Mat &image);

  void mouseLaserHandler(int event, int x, int y, int flags, const cv::Mat &image);

private:
  // Data bridge variables
  laser_geometry::LaserProjection projector_;
  sensor_msgs::PointCloud2 cloud;

  // Data paths
  std::string output_file;
  unsigned int im_count;

  // Visualization and data mapping
  LaserDisplay *ldisplay; // Visualize and map laser data
  cv::Rect rect_pannel;
  bool there_is_new_line;
  cv::Mat ploting_image; // Keep image to plot image lines

  // Features
  std::vector< std::vector<cv::Vec2f> > im_lines; // Image Lines
  cv::Vec2f current_camera_line;
  std::vector< cv::Mat> laser_scans; // Image Lines

  // Methods
  bool detectLine(const cv::Mat &image, cv::Vec2f &line_detected, int x, int y);
  /*
          This functions create a txt file that contains the panel lines
          recorded and the point cloud_ matrixes.

          n (Number of features)
          line[0][0][0] line[0][0][1] line[0][1][0] line[0][1][1] line[0][2][0] line[0][2][1]
          .
          .
          .
          line[n-1][0][0] line[n-1][0][1] line[n-1][1][0] line[n-1][1][1] line[n-1][2][0] line[n-1][2][1]
          m (number of 2d points) p[0][0][0] p[0][1][0]...p[0][m-1][0] p[0][0][1] p[0][0][1]...p[0][m-1][1]
          .
          .
          .
          m (number of 2d points) p[n-1][0][0] p[n-1][1][0]...p[n-1][m-1][0] p[n-1][0][1] p[n-1][0][1]...p[n-1][m-1][1]
  */
  void save(std::string foutput);
};

/******************************************************************************/
/*                                Classifier                                  */
/******************************************************************************/
class Classifier
{
public:
  Classifier(std::string svm_model_file);

  float predict(const cv::Mat &image);

private:
  cv::HOGDescriptor HOG_; // Use standard parameters here
  cv::Size trainingPadding_;
  cv::Size winStride_;
  CvSVM SVM_;

  // Methods
  void calculateHOG(const cv::Mat& image, cv::Mat &descriptor);
};


class LidarObject
{
public:
  LidarObject()
  {
    min_x = 1E+37;
    max_x = -1E+37;
    min_y = 1E+37;
    max_y = -1E+37;
    width = 0;
    height = 0;
    label = -1;
  };


  std::vector<int> idxs;
  float min_x, max_x, min_y, max_y;
  int min_x_idx, max_x_idx, min_y_idx, max_y_idx;
  float width, height;
  cv::Point2f centroid;
  unsigned int label;
};

/******************************************************************************/
/*                           LidarObstacleTracker                             */
/******************************************************************************/
class LidarObstacleTracker
{
public:
  LidarObstacleTracker();
  virtual ~LidarObstacleTracker();

  /*
   * This function get all the objects and track them by calculating the
   * distances between the current and the previous scan.
   */
  void track(std::vector<LidarObject> &objects, std::vector<int> &labels, float thres);

  /*
   * This function initialize the object list that keeps information about
   * the number of points of each objects and its size.
   */
  static std::vector<LidarObject> initObjectList(const pcl::PointCloud<pcl::PointXY> &points2d, const std::vector<int> &labels, int n_segments);

private:
  std::vector<LidarObject> _previous_scan;

  // Mathods
  float calculateDistance(const cv::Point2f &p1, const  cv::Point2f &p2);

};

/******************************************************************************/
/*                               SensorFusion                                 */
/******************************************************************************/
class SensorFusion
{
public:
  SensorFusion();
  SensorFusion(ros::NodeHandle &nh);
  virtual ~SensorFusion();

  void callback(const sensor_msgs::LaserScan::ConstPtr& scan, const sensor_msgs::ImageConstPtr& msg);

private:
  // Node Handler
  ros::NodeHandle nh_;

  // Publisher
  ros::Publisher labels_publisher_;
  ros::Publisher pedestrian_publisher_;

  // Data bridge variables
  laser_geometry::LaserProjection projector_;
  sensor_msgs::PointCloud2 cloud_;

  // Sensor fusion utilities
  LidarImageConverter *lic_;
  LaserSegmentation *ls_;
  LidarObstacleTracker object_tracker;

  // Classifier
  Classifier *classifier_;
  std::vector<unsigned int> pedestrian_label;

  // Visualization variables
  LaserDisplay *laser_display_;

  // Time measurement variable
  int64 work_begin_;
  double work_fps_;

  // Methods
  void fromPCL2Camera(const pcl::PointCloud<pcl::PointXY> &points2d,std::vector<cv::Point2f> &camera_points);
  void remove0Range2DPoints(const sensor_msgs::LaserScanConstPtr& scan,pcl::PointCloud<pcl::PointXY> &points2d);

  /*
   * This function gets a Lidar Object List and return a list of bounding boxes.
   */
  std::vector<cv::Rect> getBoundingBoxes(const std::vector<LidarObject> &objectList, const std::vector<cv::Point2f> &camera_points);

  // Publishers
  void publishLabels(const std::vector<int> &labels, const sensor_msgs::LaserScanConstPtr& scan);
  void publishPedestrian(const LidarObject &object, const sensor_msgs::LaserScanConstPtr& scan);

  // Time methods
  void workBegin();
  void workEnd();
  std::string workFpsToStr();

#ifdef RECORD_CAMERA
  // Record from camera
  cv::VideoWriter vw_camera;

  cv::VideoWriter vw_map;
#endif

};

}

#endif /* SENSORFUSION_H_ */
