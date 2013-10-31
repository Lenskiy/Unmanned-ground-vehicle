/*
 * SensorFusion.cpp
 *
 *  Created on: Mar 25, 2013
 *      Author: Daniel Oñoro Rubio
 */

#include "SensorFusion.h"

using namespace sf;

/******************************************************************************/
/*                               SensorFusion                                 */
/******************************************************************************/
SensorFusion::SensorFusion()
{
}

SensorFusion::SensorFusion(ros::NodeHandle &nh)
{
  std::string laser_topic, camera_topic, lidar_labels_topic_out, calibration_file, svm_model_file, pedestrian_topic_out;
  cv::Mat K, R, t;
  double max_angle, min_dist, max_dist, grid_size;

  nh_ = nh;

  // Getting parameters
  nh.param("laser_topic", laser_topic, std::string("/scan"));
  nh.param("camera_topic", camera_topic, std::string("/image_raw"));
  nh.param("calibration_url", calibration_file, std::string("config_file.yaml"));
  nh.param("svm_model_url", svm_model_file, std::string("svm_model.yaml"));
  nh.param("pedestrian_topic", pedestrian_topic_out, std::string("/pedestrian_position"));
  nh.param("lidar_max_angle", max_angle, 1.65806281567);
  nh.param("lidar_min_dist", min_dist, 0.0);
  nh.param("lidar_max_dist", max_dist, 81.0);
  nh.param("grid_size", grid_size, 0.4);
//  nh.param("output", output_files_names, string("output"));

  // Debug!!
  laser_topic = "/bumper_laser/scan";
  camera_topic = "/mono_cam/image_rect_color";
  calibration_file = "/home/dani/catkin_ws/src/kut_ugv_sensor_fusion/calib/calib.yaml";
  svm_model_file = "/home/dani/catkin_ws/src/kut_ugv_sensor_fusion/calib/svm_model.yaml";

  lidar_labels_topic_out = laser_topic + "/labels";
  labels_publisher_ = nh_.advertise<kut_ugv_sensor_fusion::lidar_segmentation_labels>(lidar_labels_topic_out, 1000);
  pedestrian_publisher_ = nh_.advertise<kut_ugv_msgs::WorldObject>(pedestrian_topic_out, 1000);

  ls_ = new LaserSegmentation(max_angle, min_dist, max_dist, grid_size);
  lic_ = new LidarImageConverter(calibration_file);
  laser_display_ = new LaserDisplay(max_angle, min_dist, max_dist, grid_size);
  classifier_ = new Classifier(svm_model_file);
}

SensorFusion::~SensorFusion()
{
  delete lic_;
  delete ls_;
  delete laser_display_;
  delete classifier_;
}

void SensorFusion::callback(const sensor_msgs::LaserScanConstPtr& scan, const sensor_msgs::ImageConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXY> points2d;
  std::vector<cv::Point2f> camera_points;

  // Convert from Sick to 2D point cloud_
  projector_.projectLaser(*scan, cloud_);
  pcl::fromROSMsg(cloud_, points2d);

  // Calculate fps
  workBegin();

  // Convert from lidar to camera points
  fromPCL2Camera(points2d, camera_points);

//  // Segment lidar data
  std::vector<int> labels;
  unsigned int n_segments = 0;
  ls_->process(points2d, labels,n_segments);
//  ls_->diff_segmentation(*scan,labels,n_segments,0.3,0);

// Get bounding boxes
// Filter objects
  std::vector<LidarObject> objectList;
  std::vector<cv::Rect> bbList;

  objectList = object_tracker.initObjectList(points2d, labels, n_segments);
  object_tracker.track(objectList, labels, 1.5);

  publishLabels(labels, scan);

  // Find Bounding Boxes
  bbList = getBoundingBoxes(objectList, camera_points);

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

  // Get image size
  cv::Size imSize = cv_ptr->image.size();
  // Filter BB out of the image
  std::vector<unsigned int> bb_inside_image_idx;
  for (unsigned int i = 0; i < bbList.size(); i++)
  {
    cv::Rect r = bbList[i];

    if ( !( (r.x < 0) || ((r.x + r.width) > imSize.width) || (r.y < 0) || ((r.y + r.height) > imSize.height)
        || (r.width == 0) || (r.height == 0) ) )
    {
      // Delete all BB that are out of the image
      bb_inside_image_idx.push_back(i);
    }
  }

  // Classify images
  std::vector<unsigned int> pedestrian_idx;
  for (unsigned int i = 0; i < bb_inside_image_idx.size(); i++)
  {
    cv::Mat roi(cv_ptr->image, bbList[ bb_inside_image_idx[i] ]);

    float response = classifier_->predict(roi);

    if (response == 1)
    {
      pedestrian_idx.push_back( bb_inside_image_idx[i] );
      pedestrian_label.push_back( objectList[ bb_inside_image_idx[i] ].label );
    }
  }

  // Plot lidar points
  for (unsigned int i = 0; i < camera_points.size(); i++)
  {
    cv::circle(cv_ptr->image, camera_points[i], 3, laser_display_->getLabelColor(labels[i]), -1);
  }

  // Track pedestrians
  for (unsigned int i = 0; i < pedestrian_label.size(); i++)
  {
    bool is_label_matching = false;
    unsigned int match_idx;
    for(unsigned int j = 0; j < objectList.size(); j++)
    {
      if(pedestrian_label[i] == objectList[j].label)
      {
        is_label_matching = true;
        match_idx = j;
        break;
      }
    }

    if(is_label_matching)
    {
      cv::rectangle(cv_ptr->image, bbList[match_idx], laser_display_->getLabelColor(objectList[match_idx].label), 2);
      cv::putText(cv_ptr->image, "Pedestrian", cv::Point(bbList[match_idx].x, bbList[match_idx].y - 5),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 100, 0), 2);

      // Publish data
      publishPedestrian(objectList[match_idx], scan);
    }
    else
    {
      pedestrian_label.erase( pedestrian_label.begin() + i);
      i--;
    }
  }

  // Plot pedestrian text
  for (unsigned int i = 0; i < pedestrian_idx.size(); i++)
  {
    cv::rectangle(cv_ptr->image, bbList[pedestrian_idx[i]], laser_display_->getLabelColor(objectList[pedestrian_idx[i]].label), 2);

    cv::putText(cv_ptr->image, "Pedestrian", cv::Point(bbList[pedestrian_idx[i]].x, bbList[pedestrian_idx[i]].y - 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 200, 0), 2);
  }

  // Calculate fps
  workEnd();

  // Plot fps info
  cv::putText(cv_ptr->image, "Fps: " + workFpsToStr(), cv::Point(5, 25), cv::FONT_HERSHEY_SIMPLEX, 1.0,
              cv::Scalar(255, 100, 0), 2);

  cv::Mat lidar_map = laser_display_->getMap(points2d, labels);
  cv::resize(lidar_map, lidar_map, cv::Size(), 2.5, 2.5, cv::INTER_LANCZOS4);
  cv::imshow(WINDOW_LASER_NAME, lidar_map);

  cv::imshow(WINDOW_CAMERA_NAME, cv_ptr->image);
  cv::waitKey(3);

#ifdef RECORD_CAMERA
  if (!vw_camera.isOpened())
  {
    vw_camera.open("/home/dani/camera.avi", CV_FOURCC('x', 'v', 'i', 'd'), FPS, cv_ptr->image.size(), true);

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

  if (!vw_map.isOpened())
  {
    vw_map.open("/home/dani/map.avi", CV_FOURCC('x', 'v', 'i', 'd'), FPS, lidar_map.size(), true);

    if (!vw_map.isOpened())
    {
      std::cout << "Can't create video writer. Nothing will be recorded" << std::endl;
    }
  }
  else
  {
    // Save frame
    vw_map << lidar_map;
  }
#endif
}

void SensorFusion::publishLabels(const std::vector<int> &labels, const sensor_msgs::LaserScanConstPtr& scan)
{
  kut_ugv_sensor_fusion::lidar_segmentation_labels publish_me;

  publish_me.header.frame_id = scan->header.frame_id;
  publish_me.header.stamp = scan->header.stamp;

  for(unsigned int i = 0; i < labels.size(); i++)
  {
    publish_me.labels.push_back(labels[i]);
  }

  labels_publisher_.publish(publish_me);
}

void SensorFusion::publishPedestrian(const LidarObject &object, const sensor_msgs::LaserScanConstPtr& scan)
{
  kut_ugv_msgs::WorldObject publish_me;

  publish_me.header.frame_id = scan->header.frame_id;
  publish_me.header.stamp = scan->header.stamp;
  publish_me.pose.position.x = -object.centroid.y;
  publish_me.pose.position.y = object.centroid.x;
  publish_me.pose.position.z = 0;
  publish_me.id = object.label;

  pedestrian_publisher_.publish(publish_me);
}

std::vector<cv::Rect> SensorFusion::getBoundingBoxes(const std::vector<LidarObject> &objectList,
                                                     const std::vector<cv::Point2f> &camera_points)
{
  std::vector<cv::Rect> bbList;
  bbList.resize(objectList.size());

  for (int i = 0; i < objectList.size(); i++)
  {
//    if (objectList[i].width > car_width_bot_thres && objectList[i].width < car_width_top_thres
//        && objectList[i].height > car_height_top_thres && objectList[i].height < car_height_top_thres)
//    {
//      cv::Mat top_left_corner_mat, bot_right_corner_mat;
//    }

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

      // Give squared BB
//      // Increase BB size
//      int x_inc, y_inc; // Increments
//      x_inc = r.width * 0.25 + 10;
//
//      r.x -= x_inc;
//      r.width += 2 * x_inc;
//      r.height += r.width;
//      r.y -= r.height * 0.60;

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

std::vector<LidarObject> LidarObstacleTracker::initObjectList(const pcl::PointCloud<pcl::PointXY> &points2d,
                                                      const std::vector<int> &labels, int n_segments)
{
  std::vector<LidarObject> objectList;
  objectList.resize(n_segments);

  // First pass of two. Init max/min and points.
  for (int i = 0; i < points2d.size(); i++)
  {
    LidarObject lobj = objectList[ labels[i] % n_segments ];
    lobj.idxs.push_back(i);

    if (lobj.max_x < points2d[i].x)
    {
      lobj.max_x = points2d[i].x;
      lobj.max_x_idx = i;
    }
    if (lobj.max_y < points2d[i].y)
    {
      lobj.max_y = points2d[i].y;
      lobj.max_y_idx = i;
    }
    if (lobj.min_x > points2d[i].x)
    {
      lobj.min_x = points2d[i].x;
      lobj.min_x_idx = i;
    }
    if (lobj.min_y > points2d[i].y)
    {
      lobj.min_y = points2d[i].y;
      lobj.min_y_idx = i;
    }

    lobj.label = labels[i];
    objectList[ labels[i] % n_segments] = lobj;
  }

  // Calculate width and height
  for (int i = 0; i < objectList.size(); i++)
  {
    objectList[i].height = objectList[i].max_y - objectList[i].min_y;
    objectList[i].width = objectList[i].max_x - objectList[i].min_x;

    // Calculate centroid
    int n_points = objectList[i].idxs.size();
    cv::Point2f centroid(0, 0);
    for (int j = 0; j < n_points; j++)
    {
      centroid.x += points2d[objectList[i].idxs[j]].x;
      centroid.y += points2d[objectList[i].idxs[j]].y;
    }
    centroid.x /= n_points;
    centroid.y /= n_points;

    objectList[i].centroid = centroid;

  }

  return objectList;
}

void SensorFusion::fromPCL2Camera(const pcl::PointCloud<pcl::PointXY> &points2d,
                                  std::vector<cv::Point2f> &camera_points)
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

    cv::Point2f p = lic_->from3d2image(Pl);

    // Save 2D image point
    camera_points.push_back(p);
  }
}

inline void SensorFusion::workBegin()
{
  work_begin_ = cv::getTickCount();
}

inline void SensorFusion::workEnd()
{
  int64 delta = cv::getTickCount() - work_begin_;
  double freq = cv::getTickFrequency();
  work_fps_ = freq / delta;
}

inline std::string SensorFusion::workFpsToStr()
{
  std::stringstream ss;
  ss << work_fps_;
  return ss.str();
}

/******************************************************************************/
/*                            LidarImageConverter                             */
/******************************************************************************/
LidarImageConverter::~LidarImageConverter()
{
}

LidarImageConverter::LidarImageConverter(const cv::Mat& _K, const cv::Mat& _R, const cv::Mat& _t)
{
  _K.copyTo(K);
  _R.copyTo(R);
  _t.copyTo(t);
}

LidarImageConverter::LidarImageConverter(std::string calibration_file)
{
  loadCalibrationParams(calibration_file);
}

cv::Point2f LidarImageConverter::from3d2image(const cv::Mat& Pl)
{
  cv::Mat Pi; // Point image
  double w;

  Pi = K * (R * Pl + t);
  w = Pi.at<double>(2, 0);

  return cv::Point2f(Pi.at<double>(0, 0) / w, Pi.at<double>(1, 0) / w);
}

cv::Point2f LidarImageConverter::from3d2image(const cv::Point3f& Pw)
{
  cv::Mat Pi; // Point image
  double w;

  double data_point[3];
  data_point[0] = Pw.x;
  data_point[1] = Pw.y;
  data_point[2] = Pw.z;
  cv::Mat PwMat(3, 1, CV_64F, data_point); // Point 3d image

  Pi = K * (R * PwMat + t);
  w = Pi.at<double>(2, 0);

  return cv::Point2f(Pi.at<double>(0, 0) / w, Pi.at<double>(1, 0) / w);
}

bool LidarImageConverter::loadCalibrationParams(std::string file)
{
  cv::FileStorage fs2(file, cv::FileStorage::READ);

  ROS_INFO("Calibration path: %s\n", file.c_str());

  if (!fs2.isOpened())
  {
    ROS_ERROR("Could not open Calibration file.\n");
    return false;
  }

  fs2["K"] >> K;
  fs2["R"] >> R;
  fs2["t"] >> t;

  fs2.release();

  return true;
}

/******************************************************************************/
/*                             LaserSegmentation                              */
/******************************************************************************/
LaserSegmentation::LaserSegmentation(double _max_angle, double _min_dist, double _max_dist, double _grid_size)
{
  bin_size = _grid_size;
  double min_dist; // If the angle is over pi/2

  /*
   * If it is over 90º the minimun distance in x axis will be 0 and the
   * maximun distance in y axis corresponds to sin(90º) which is 1.
   */
  if (_max_angle > PI / 2)
  {
    min_dist = _max_dist * cos(_max_angle);
    map_height = 2 * _max_dist / bin_size;
  }
  else
  {
    min_dist = _min_dist;
    map_height = 2 * _max_dist * sin(_max_angle) / bin_size;
  }
  map_width = (abs(min_dist) + _max_dist) / bin_size;

  origin.x = abs(min_dist) / bin_size;
  origin.y = map_height / 2;

  // Create matrix
  map = new grid_bin*[map_width];
  for (int i = 0; i < map_width; i++)
  {
    map[i] = new grid_bin[map_height];
  }

  current_label = 0;

}

LaserSegmentation::~LaserSegmentation()
{
  // Dispose memory
  for (int i = 0; i < map_width; i++)
  {
    delete[] map[i];
  }
  delete[] map;
}

void LaserSegmentation::process(const pcl::PointCloud<pcl::PointXY> &points2d, std::vector<int> &labels,
                                unsigned int &n_segments)
{
  unsigned int x, y;
  unsigned int initial_label = current_label;
  cv::Point bin_pos;
  DList<cv::Point> neighborhood;

  labels.clear(); // Clean vector, just for security.
  labels.resize(points2d.size());

  /*
   * 1 - Fill map. Place every point in its correspondent bin
   */
  for (size_t i = 0; i < points2d.size(); i++)
  {
    x = (unsigned int)(points2d[i].x / bin_size + origin.x);
    y = (unsigned int)(points2d[i].y / bin_size + origin.y);

    if (map[x][y].isEmpty)
    {
      /*
       * Save a pointer of the element, in other to clean the map at the end of
       * the algorithm.
       */
      ocupated_bins_list.push_back(&map[x][y]);

      // Save the coordinates of this bin.
      bin_position_list.push_back(cv::Point(x, y));
      // Save point to bin_position_list node in other to go forward.
      map[x][y].ptr_bin_position_list = bin_position_list.end();
      // Bin not empty
      map[x][y].isEmpty = false;
    }
    // Set values
    map[x][y].points_list.push_back(i);
  }

  /*
   * 2 - Start labeling elements using a connected components algorithm.
   */
  while (!bin_position_list.empty())
  {
    bin_pos = bin_position_list.front();

    // Label element
    label_element(bin_pos);

    // Get the occupied neighborhood of the current element.
    get_neighborhood(bin_pos, neighborhood);

    while (!neighborhood.empty())
    {
      bin_pos = neighborhood.front();
      neighborhood.pop_front();

      // Label element
      label_element(bin_pos);

      // Get the occupied neighborhood of the current element.
      get_neighborhood(bin_pos, neighborhood);
    }

    // Shaped totally labeled. Update current_label.
    current_label++;
  }

  // Return labels and clean map and memory
  while (!ocupated_bins_list.empty())
  {
    grid_bin* bin = ocupated_bins_list.front();
    ocupated_bins_list.pop_front();

    while (!bin->points_list.empty())
    {
      labels[bin->points_list.front()] = bin->label;
      bin->points_list.pop_front();
    }

    // Dispose bin
    bin->isEmpty = true;
    bin->hasLabel = false;
  }

  n_segments = current_label - initial_label;
}

void LaserSegmentation::get_neighborhood(cv::Point bin_pos, DList<cv::Point>& neighborhood)
{
  unsigned int x_ini, y_ini, x_end, y_end;

  x_ini = ((bin_pos.x - 1) > 0) ? bin_pos.x - 1 : 0;
  x_end = ((bin_pos.x + 1) < map_width) ? bin_pos.x + 1 : map_width - 1;
  y_ini = ((bin_pos.y - 1) > 0) ? bin_pos.y - 1 : 0;
  y_end = ((bin_pos.y + 1) < map_height) ? bin_pos.y + 1 : map_height - 1;

  for (unsigned int i = x_ini; i <= x_end; i++)
  {
    for (unsigned int j = y_ini; j <= y_end; j++)
    {
      if (!map[i][j].isEmpty && !map[i][j].hasLabel)
      {
        // Save the coordinates of this bin.
        neighborhood.push_back(cv::Point(i, j));
        map[i][j].hasLabel = true;
      }
    }
  }
}

inline void LaserSegmentation::label_element(cv::Point bin_pos)
{
  map[bin_pos.x][bin_pos.y].label = current_label;
  map[bin_pos.x][bin_pos.y].isEmpty = true;
  bin_position_list.erase(map[bin_pos.x][bin_pos.y].ptr_bin_position_list); // Delete to avoid to check again
}

/*
 * Calculates distance dependance between two consecutive lidar beams.
 */
float LaserSegmentation::distance_dependence(float delta_theta, float r0, float r1)
{
  return tan(delta_theta) * fmin(r0, r1);
}

void LaserSegmentation::diff_segmentation(const sensor_msgs::LaserScan& scan, std::vector<int> &labels, int &n_segments,
                                          float d_th, float bias)
{
  int n_points = scan.ranges.size();
  unsigned int cur_label = 0;
  float d_th_x;

  if (scan.ranges.empty())
  {
    ROS_ERROR("No data from lidar received\n");

  }

  // Clean and resize labels
  labels.clear(); // Clean vector, just for security.
  labels.resize(n_points);

  // Init number of segments
  n_segments = 0;

  for (int i = 0; i < (n_points - 1); i++)
  {
    d_th_x = bias + distance_dependence(scan.angle_increment, scan.ranges[i], scan.ranges[i + 1]);

    if (d_th_x >= d_th)
    {
      cur_label++;
    }

    labels[i] = cur_label;
  }

  // Return value
  n_segments = cur_label;
}

/******************************************************************************/
/*                           LidarObstacleTracker                             */
/******************************************************************************/
LidarObstacleTracker::LidarObstacleTracker()
{

}

LidarObstacleTracker::~LidarObstacleTracker()
{

}

void LidarObstacleTracker::track(std::vector<LidarObject> &objects, std::vector<int> &labels, float thres)
{
  // Correct labels associating them to the nearest neigbour
  for (unsigned int i = 0; i < _previous_scan.size(); i++)
  {
    unsigned int last_idx = 0;
    float dist = calculateDistance(objects[0].centroid, _previous_scan[i].centroid);

    for (unsigned int j = 1; j < objects.size(); j++)
    {
      float current_dist = calculateDistance(objects[j].centroid, _previous_scan[i].centroid);

      if (current_dist < dist)
      {
        last_idx = j;
        dist = current_dist;
      }
    }

//    printf("Dist NN: %f\n",dist);

    // Check threshold
    if (dist < thres)
    {
      int label_val = _previous_scan[i].label;
      objects[last_idx].label = label_val;
      // Update labels
      for (unsigned int k = 0; k < objects[last_idx].idxs.size(); k++)
      {
//        printf("P/C label: [%d,%d]\n", label_val, labels[ objects[last_idx].idxs[k] ]);

        labels[objects[last_idx].idxs[k]] = label_val;
      }
    }
  }

  // Update previous scan
  _previous_scan = std::vector<LidarObject>(objects);
}

float inline LidarObstacleTracker::calculateDistance(const cv::Point2f &p1, const cv::Point2f &p2)
{
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

/******************************************************************************/
/*                          CalibrationFeaturesTool                           */
/******************************************************************************/
LaserDisplay::LaserDisplay(double _max_angle, double _min_dist, double _max_dist, double _grid_size)
{
  bin_size = _grid_size;
  double min_dist; // If the angle is over pi/2

  /*
   * If it is over 90º the minimun distance in x axis will be 0 and the
   * maximun distance in y axis corresponds to sin(90º) which is 1.
   */
  if (_max_angle > PI / 2)
  {
    min_dist = _max_dist * cos(_max_angle);
    map_height = 2 * _max_dist / bin_size;
  }
  else
  {
    min_dist = _min_dist;
    map_height = 2 * _max_dist * sin(_max_angle) / bin_size;
  }
  map_width = (abs(min_dist) + _max_dist) / bin_size;

  origin.x = abs(min_dist) / bin_size;
  origin.y = map_height / 2;

  // Create matrix
  map = new display_bin*[map_width];
  for (int i = 0; i < map_width; i++)
  {
    map[i] = new display_bin[map_height];
  }

  // Set visualization variables
  white_color = cv::Vec3b(255, 255, 255);

  for (unsigned int i = 0; i < MAX_COLOURS; i++)
  {
    colour_vector_.push_back(cv::Vec3b(rand() & 255, rand() & 255, rand() & 255));
  }
}

LaserDisplay::~LaserDisplay()
{
}

cv::Mat LaserDisplay::getMap(const pcl::PointCloud<pcl::PointXY> &points2d, const std::vector<int> &labels)
{
  unsigned int x, y;
  cv::Point bin_pos;

  // Create image
  img_map = cv::Mat::zeros(map_width, map_height, CV_8UC3);
  //  cv::line(img_map,cv::Point(0, map_width - origin.x),cv::Point(img_map.size().width, map_width - origin.x),cv::Scalar(0,0,255));
  //  cv::line(img_map,cv::Point(origin.y,0),cv::Point(origin.y,img_map.size().height),cv::Scalar(0,0,255));
  cv::line(img_map, cv::Point(0, origin.x), cv::Point(img_map.size().width, origin.x), cv::Scalar(0, 0, 255));
  cv::line(img_map, cv::Point(origin.y, 0), cv::Point(origin.y, img_map.size().height), cv::Scalar(0, 0, 255));

  // Clear map
  while (!ocupated_bins_list.empty())
  {
    display_bin* bin = ocupated_bins_list.front();
    ocupated_bins_list.pop_front();

    bin->points_list.clear();

    // Dispose bin
    bin->isEmpty = true;
  }

  /*
   * 1 - Fill map. Place every point in its correspondent bin
   */
  for (size_t i = 0; i < points2d.size(); i++)
  {
    x = (unsigned int)(points2d[i].x / bin_size + origin.x);
    y = (unsigned int)(points2d[i].y / bin_size + origin.y);

    if (map[x][y].isEmpty)
    {
      /*
       * Save a pointer of the element, in other to clean the map at the end of
       * the algorithm.
       */
      ocupated_bins_list.push_back(&map[x][y]);

      // Save the coordinates of this bin.
      bin_position_list.push_back(cv::Point(x, y));
      // Bin not empty
      map[x][y].isEmpty = false;

      // Plot from a 180º rotated version
      //      img_map.at<cv::Vec3b>(map_width - x,map_height - y) = white_color;
      if (labels.empty())
      {
        img_map.at<cv::Vec3b>(x, y) = white_color;
      }
      else
      {
        int label_idx = labels[i] % MAX_COLOURS;
        if (label_idx >= 0)
        {
          img_map.at<cv::Vec3b>(x, y) = colour_vector_[ label_idx ];
        }
        else
        {
          img_map.at<cv::Vec3b>(x, y) = white_color;
        }
      }
    }
    // Set values
    map[x][y].points_list.push_back(points2d[i]);
  }

  return img_map;
}

cv::Scalar LaserDisplay::getLabelColor(int label_idx)
{
  cv::Scalar ret;
  int label = label_idx % MAX_COLOURS;

  if (label >= 0)
  {
    ret = cv::Scalar(colour_vector_[ label ]);
  }
  else
  {
    ret = cv::Scalar(white_color);
  }

  return ret;
}

cv::Mat LaserDisplay::getRoiPoints(cv::Rect roi_area)
{
  std::vector<pcl::PointXY> point_list;

  // Get data
  // Convert data to current coordinate frame
  int ini_x = roi_area.x; //map_width - roi_area.y;
  int ini_y = roi_area.y; //map_height - roi_area.x;

  for (int ix = ini_x; ix < (ini_x + roi_area.width); ix++)
  {
    for (int iy = ini_y; iy < (ini_y + roi_area.height); iy++)
    {

      display_bin bin = map[iy][ix];
      std::vector<pcl::PointXY> bin_p_list = bin.points_list;

      // Get data
      for (int i = 0; i < bin_p_list.size(); i++)
      {
        pcl::PointXY p = bin_p_list[i];
        point_list.push_back(p);
      }
    }
  }

  // Convert to matrix
  unsigned int n_points = point_list.size();
  cv::Mat output = cv::Mat(n_points, 2, CV_32FC1);

  for (unsigned int i = 0; i < n_points; i++)
  {
    pcl::PointXY point = point_list[i];
    output.at<float>(i, 0) = point.x;
    output.at<float>(i, 1) = point.y;
  }

  return output;
}

CalibrationFeaturesTool::CalibrationFeaturesTool()
{
  output_file = "output.txt";
  im_count = 0;
}

CalibrationFeaturesTool::~CalibrationFeaturesTool()
{
  delete ldisplay;

  // Save data
  save(output_file);
}

bool CalibrationFeaturesTool::setUp(std::string input_file, std::string _output_file)
{
  cv::Mat K;
  cv::Mat R;
  cv::Mat t;
  double max_angle, min_dist, max_dist, grid_size;

  // Set up output file path
  output_file = _output_file;

  cv::FileStorage fs2(input_file, cv::FileStorage::READ);

  std::cout << "\n\nConfiguration path: " << input_file << std::endl;

  if (!fs2.isOpened())
  {
    return false;
  }

  fs2["max_angle"] >> max_angle;
  fs2["min_dist"] >> min_dist;
  fs2["max_dist"] >> max_dist;
  fs2["grid_size"] >> grid_size;

  fs2.release();

  ldisplay = new LaserDisplay(max_angle, min_dist, max_dist, grid_size);

  return true;
}

bool CalibrationFeaturesTool::detectLine(const cv::Mat &image, cv::Vec2f &line_detected, int x, int y)
{
  cv::Size im_s = image.size();

  // Calculate a region of interest (ROI) which is fully inside the image
  cv::Rect roi_area(x - ROI_WIDTH / 2, y - ROI_HEIGHT / 2, ROI_WIDTH, ROI_HEIGHT);
  roi_area.x = (roi_area.x < 0) ? 0 : roi_area.x;
  roi_area.x = ((roi_area.x + ROI_WIDTH) < im_s.width) ? roi_area.x : im_s.width - ROI_WIDTH;
  roi_area.y = (roi_area.y < 0) ? 0 : roi_area.y;
  roi_area.y = ((roi_area.y + ROI_HEIGHT) < im_s.height) ? roi_area.y : im_s.height - ROI_HEIGHT;

  // Set roi
  cv::Mat roi(image, roi_area);
  roi.copyTo(roi); // Create a separated copy

  // Convert to gray
  cv::cvtColor(roi, roi, CV_BGR2GRAY);

  // Extract edges
  cv::Mat roi_bin;
  cv::Canny(roi, roi_bin, 10, 200);

  // Find lines
  std::vector<cv::Vec4i> lines; // Hough segments
  cv::Vec2f avg_line(0, 0); // Polar lines
  cv::HoughLinesP(roi_bin, lines, 1, CV_PI / 180, 20, 30, 10);

  unsigned int n_lines = lines.size();
  printf("N_lines: %d\n", n_lines);

  if (!n_lines)
  {
    printf("No lines detected!!\n");
    return false;
  }

  for (size_t i = 0; i < lines.size(); i++)
  {
    // Convert to image coordinates
    lines[i][0] += roi_area.x;
    lines[i][1] += roi_area.y;
    lines[i][2] += roi_area.x;
    lines[i][3] += roi_area.y;

    float m, b;

    // Avoid 0 division by adding a pixel of error
    lines[i][2] = (lines[i][2] == lines[i][0]) ? lines[i][2] + 1 : lines[i][2];

    m = ((float)(lines[i][3] - lines[i][1])) / ((float)(lines[i][2] - lines[i][0]));

    float theta = fabs(atan(m)) * 180 / CV_PI;

    // Add only vertical lines
    if (theta > 80 && theta < 100)
    {
      // Add line
      b = (float)lines[i][1] - m * (float)lines[i][0];

      avg_line[0] += m;
      avg_line[1] += b;
    }
    else
    {
      // Filter out line
      n_lines--;
    }

  }

  if (n_lines <= 0)
  {
    printf("No vertical lines detected!!\n");
    return false;
  }

  avg_line[0] = avg_line[0] / n_lines;
  avg_line[1] = avg_line[1] / n_lines;

  // Return line
  line_detected = avg_line;

  return true;
}

struct WrappedParamOnMouse
{
  CalibrationFeaturesTool *cft;
  cv::Mat *image;
  char method;
};

// In other to use classes with setMouseCallback function
void wrappedOnMouse(int event, int x, int y, int flags, void* ptr)
{
  WrappedParamOnMouse *param = (WrappedParamOnMouse*)ptr;
  CalibrationFeaturesTool* cftPtr = param->cft;
  cv::Mat image = *param->image;

  if (cftPtr != NULL)
  {
    switch (param->method)
    {
      case 'c':
        cftPtr->mouseCameraHandler(event, x, y, flags, image);
        break;
      case 'l':
        cftPtr->mouseLaserHandler(event, x, y, flags, image);
    }
  }
}

void CalibrationFeaturesTool::mouseLaserHandler(int event, int x, int y, int flags, const cv::Mat &image)
{
  if (event == cv::EVENT_LBUTTONDOWN)
  {
    cv::Size im_s = image.size();
    cv::Mat ploting_image;
    image.copyTo(ploting_image);

    // Calculate a region of interest (ROI) which is fully inside the image
    cv::Rect roi_area(x - ROI_LASER / 2, y - ROI_LASER / 2, ROI_LASER, ROI_LASER);

    roi_area.x = (roi_area.x < 0) ? 0 : roi_area.x;
    roi_area.x = ((roi_area.x + ROI_LASER) < im_s.width) ? roi_area.x : im_s.width - ROI_LASER;
    roi_area.y = (roi_area.y < 0) ? 0 : roi_area.y;
    roi_area.y = ((roi_area.y + ROI_LASER) < im_s.height) ? roi_area.y : im_s.height - ROI_LASER;

    cv::rectangle(ploting_image, roi_area, cv::Scalar(0, 0, 255), 1);

    rect_pannel = roi_area;

    cv::imshow(WINDOW_LASER_LINES_NAME, ploting_image);
    printf("clic_possition[%d,%d]\n", x, y);
    printf("Press 'A' to find panel in this region.\nPress Esc to exit.\n");
  }
}

void CalibrationFeaturesTool::mouseCameraHandler(int event, int x, int y, int flags, const cv::Mat &image)
{
  cv::Size im_s = image.size();
  // Plot panel lines
  std::vector<cv::Vec2f> panel_lines = im_lines.back();
  for (unsigned int i = 0; i < panel_lines.size(); i++)
  {
    cv::Vec2f line = panel_lines[i];
    cv::line(ploting_image, cv::Point2f(-line[1] / line[0], 0),
             cv::Point2f((im_s.height - line[1]) / line[0], im_s.height), cv::Scalar(0, 0, 255), 1, 8);
  }

  if (event == cv::EVENT_LBUTTONDOWN)
  {
    cv::Vec2f line_detected;
    image.copyTo(ploting_image);

    // Detect lines
    if (!detectLine(image, current_camera_line, x, y))
    {
      return;
    }
    there_is_new_line = true;

    // Plot current line
    cv::line(ploting_image, cv::Point2f(-current_camera_line[1] / current_camera_line[0], 0),
             cv::Point2f((im_s.height - current_camera_line[1]) / current_camera_line[0], im_s.height),
             cv::Scalar(255, 0, 0), 1, 8);

    printf("clic_possition[%d,%d]\n", x, y);
    printf("Press 'A' to add this line or click again to find a new one.\nPress Esc to exit.\n");
  }

  cv::imshow(WINDOW_CAMERA_LINES_NAME, ploting_image);
}

void CalibrationFeaturesTool::save(std::string foutput)
{
  foutput += ".txt"; // Add extension
  //Out file
  std::ofstream ffile(foutput.c_str());

  int n = im_lines.size();

  if (ffile.is_open())
  {
    //Write number of features
    ffile << n << std::endl;

    //Write lines
    for (int i = 0; i < n; i++)
    {
      for (int j = 0; j < im_lines[i].size(); j++)
      {
        cv::Vec2f line = im_lines[i][j];
        ffile << line[0] << " " << line[1] << " ";
      }
      ffile << std::endl;
    }

    //Write point clouds
    for (int i = 0; i < n; i++)
    {
      cv::Mat points = laser_scans[i];
      cv::Size s = points.size();

      int m = s.height;

      // Save number of points
      ffile << m << " ";

      for (int j = 0; j < s.width; j++)
      {
        for (int k = 0; k < m; k++)
        {
          ffile << points.at<float>(k, j) << " ";
        }
      }
      ffile << std::endl;
    }
  }
  else
  {
    std::cerr << "Error: could not save file." << std::endl;
  }

  //close file
  ffile.close();
}

void CalibrationFeaturesTool::callback(const sensor_msgs::LaserScanConstPtr& scan,
                                       const sensor_msgs::ImageConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXY> points2d;

  // Convert from Sick to 2D point cloud_
  projector_.projectLaser(*scan, cloud);
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

  cv::Mat laserMap = ldisplay->getMap(points2d, std::vector<int>());

  char key = 0;
  cv::imshow(WINDOW_CAMERA_NAME, cv_ptr->image);
  cv::imshow(WINDOW_LASER_NAME, laserMap);

  key = cv::waitKey(3);

  // Pause video and start collecting lines
  if (key == 'p')
  {
    WrappedParamOnMouse param;
    param.cft = this;
    param.image = &cv_ptr->image;
    param.method = 'c'; // Execute camera method
    cv_ptr->image.copyTo(ploting_image);

    cv::destroyWindow(WINDOW_CAMERA_NAME);
    cv::destroyWindow(WINDOW_LASER_NAME);

    // Show frame where to click in order to find lineas
    cv::namedWindow(WINDOW_CAMERA_LINES_NAME);
    cv::setMouseCallback(WINDOW_CAMERA_LINES_NAME, wrappedOnMouse, (void*)&param);

    // Add line feature
    im_lines.push_back(std::vector<cv::Vec2f>());

    // Some flags for the loop
    bool camera_collected = false;

    // Collect lines from the camera
    do
    {
      cv::imshow(WINDOW_CAMERA_LINES_NAME, cv_ptr->image);
      key = cv::waitKey(0);

      // Add line if user press A or a
      if (key == 'a' || key == 'A')
      {
        if (there_is_new_line)
        {
          im_lines.back().push_back(current_camera_line);
          there_is_new_line = false;
        }
        else
        {
          printf("There is no new line!!\n");
        }
      }

      // If already collected all the lines of the panel, finish
      camera_collected = im_lines.back().size() >= N_LINES;

    } while (key != 27 && !camera_collected);

    // If could not collect all lines, remove all last.
    if (camera_collected)
    {

      // Show frame where to click in order to find lines
      param.image = &laserMap;
      param.method = 'l'; // Execute camera method

      cv::namedWindow(WINDOW_LASER_LINES_NAME);
      cv::setMouseCallback(WINDOW_LASER_LINES_NAME, wrappedOnMouse, (void*)&param);

      // Some flags for the loop
      bool panel_collected = false;

      // Collect points from laser
      cv::Mat panel;
      do
      {
        cv::imshow(WINDOW_LASER_LINES_NAME, laserMap);
        key = cv::waitKey(0);

        // Get points if user press A or a
        if (key == 'a' || key == 'A')
        {
          panel = ldisplay->getRoiPoints(rect_pannel);

          if (!panel.empty())
          {
            panel_collected = true;
            laser_scans.push_back(panel);
          }
          else
          {
            printf("No points in that region!!\n");
          }
        }

      } while (key != 27 && !panel_collected);

      // If data is not collected remove last line features.
      if (!panel_collected)
      {
        im_lines.pop_back();
      }
      else
      {
        // Everything was ok, so also save image
        std::stringstream ss;
        ss << output_file << ++im_count << ".jpg";
        std::vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
        compression_params.push_back(100);
        cv::imwrite(ss.str(), cv_ptr->image, compression_params);
      }
    }
    else
    {
      im_lines.pop_back();
    }

    // Destroy windows that we don't need
    cv::destroyWindow(WINDOW_LASER_LINES_NAME);
    cv::destroyWindow(WINDOW_CAMERA_LINES_NAME);

    printf("\n----------------------------\n");
    printf("   Features recorded: %d\n", im_lines.size());
    printf("----------------------------\n");
  }

}

/******************************************************************************/
/*                                Classifier                                  */
/******************************************************************************/

Classifier::Classifier(std::string svm_model_file)
{
  cv::Size trainingPadding_ = cv::Size(0, 0);
  cv::Size winStride_ = cv::Size(8, 8);

  HOG_.winSize = cv::Size(64, 128);

  SVM_.load(svm_model_file.c_str());
}

float Classifier::predict(const cv::Mat &image)
{
  cv::Mat descriptor;

  // Make sure that the image has and appropriated size
  cv::Mat scaled_image;
  cv::cvtColor(image, scaled_image, CV_BGR2GRAY);
  cv::resize(image, scaled_image, cv::Size(HOG_.winSize.width, HOG_.winSize.height));


  calculateHOG(scaled_image, descriptor);

  return SVM_.predict(descriptor);
}

void Classifier::calculateHOG(const cv::Mat& image, cv::Mat &descriptor)
{
  // Check for mismatching dimensions
  if (image.cols != HOG_.winSize.width || image.rows != HOG_.winSize.height)
  {
    ROS_ERROR("Error: Image dimensions (%u x %u) do not match HOG window size (%u x %u)!\n", image.cols, image.rows,
              HOG_.winSize.width, HOG_.winSize.height);
    return;
  }
  std::vector<cv::Point> locations;
  std::vector<float> featureVector;
  HOG_.compute(image, featureVector, winStride_, trainingPadding_, locations);

  // Convert to to OpenCV matrix format
  descriptor = cv::Mat(featureVector.size(), 1, CV_32FC1, featureVector.data());
}
