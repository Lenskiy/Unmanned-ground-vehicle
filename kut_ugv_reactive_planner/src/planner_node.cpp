#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <kut_ugv_reactive_planner/HeadingStamped.h>
#include <kut_ugv_msgs/MotionCommandStamped.h>
#include <kut_ugv_msgs/VelocityLimit.h>
#include <kut_ugv_vehicle/StateStamped.h>
#include <kut_ugv_vehicle/State.h>
#include <kut_ugv_sensor_fusion/lidar_object.h>
#include <kut_ugv_sensor_fusion/lidar_object_list.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sstream>
#include <fstream>
#include <iostream>

#define planning_sample 150
#define data_size 5
#define wheel_base 2.7
#define num_path 13
#define lat_off_rate 10
#define GPStoLaser 3.66

#define mapsize 12000
#define local_mapsize 700
#define shiftmap mapsize/2
#define cellsize 0.15
#define scale 1000

//#define wp_size 2565 // F1
//#define wp_size 2178 // KUT
#define wp_size 4600 // KATECH

static cv::Mat global_map(mapsize,mapsize,CV_8UC1,120);
static double init_gpsx;
static double init_gpsy;

double laser_data_[1141];

std::vector<tf::Point> path_;

class ReactivePlannerNode
{
public:

  cv::Mat grid_map_;
  cv::Mat global_map_;

  std::ofstream heading_file_;
  std::ofstream log_file_;
  std::vector<tf::Point> wp_;

  //std::vector<tf::Point> path_;



  std::vector<tf::Point> curvature;
  std::vector<tf::Point> ref_velocity_;


  tf::Point gps_pt_;

  double current_heading_;
  double angle_diff_;
  double curvature_;
  double lateral_offset_;
  double speed_command_;
  double crio_speed_;
  double speed_;
  double theta_;
  double beta_;

  double velocity_x_;
  double velocity_y_;
  double velocity_max_;
  double action_type_;

//  double object_height_;
//  double object_width_;
//  double object_min_x_;
//  double object_max_x_;
//  double object_min_y_;
//  double object_max_y_;
//  int object_idxs_;
//  int object_min_x_idx_;
//  int object_max_x_idx_;
//  int object_min_y_idx_;
//  int object_max_y_idx_;
//  double object_x_;
//  double object_y_;

  double narrow_path_x_;
  double narrow_path_y_;
  int narrow_path_flag_;

  tf::Point narrow_path_start_;

  double trajectory_[num_path][planning_sample][3];
  //double path_[planning_sample][2];

  double lookahead_x;
  double lookahead_y;

  double gps_sec;
  double laser_sec;
  double gps_nsec;
  double laser_nsec;

  double planner_speed_limit;
  double planner_speed_limit_x;
  double planner_speed_limit_y;

  double sample_time;
  //double old_steering_angle;

  double laser_x_[1141];
  double laser_y_[1141];

  int heading_condition;
  int32_t publish_rate_;
  std::string wp_filename_;
  std::string curvature_filename_;
  std::string velocity_filename_;
  std::string log_filename_;
  std::string log_filename2_;

  pcl::PointCloud<pcl::PointXYZ> roof_cloud_;
  pcl::PointCloud<pcl::PointXYZ> bumper_cloud_;

  ros::NodeHandlePtr node_;
  ros::NodeHandlePtr pnode_;

  ros::Publisher heading_pub_;
  ros::Publisher motion_pub_;
  ros::Publisher trajectory_pub_;
  ros::Publisher planner_velocity_pub_;
  ros::Subscriber waypoint_sub_;
  ros::Subscriber curvature_sub_;
  ros::Subscriber laser_sub_;
  ros::Subscriber gps_sub_;
  ros::Subscriber vehicle_sub_;
  ros::Subscriber roof_cloud_sub_;
  ros::Subscriber bumper_cloud_sub_;
  ros::Subscriber velocity_limit_sub_;
  ros::Subscriber object_sub_;

  kut_ugv_reactive_planner::HeadingStamped cur_steering_;

  //int iteration_;
  ReactivePlannerNode() :
    current_heading_(0.0), publish_rate_(1000)
  {
    //    this->init();
  }
  ~ReactivePlannerNode()
  {
    heading_file_.close();
  }


  void dotTogrid(double x, double y, int& xgrid, int& ygrid){
          xgrid = ((int)(x / cellsize))  + shiftmap;
          ygrid = ((int)(y / cellsize))  + shiftmap;
  }

  inline cv::Mat translate(double x, double y)
  {
   cv::Mat T;

   T = (cv::Mat_<double >(4,4) <<  1,    0,  0,     x,
          0,    1,  0,     y,
          0,   0,  1,    0,
             0,     0,  0,   1);
   return T;

  }

  inline cv::Mat rotate(double theta)
  {
   cv::Mat R;

   R = (cv::Mat_<double >(4,4) <<  cos(theta),    -sin(theta),  0,     0,
                                 sin(theta),    cos(theta),  0,     0,
                                     0,   0,  1,    0,
                                     0,     0,  0,   1);
   return R;

  }



  inline cv::Mat convmat(double x,double y,double theta){

    cv::Mat A(4,4,CV_64FC1), R(4,4,CV_64FC1), T(4,4,CV_64FC1);
    A.zeros(4,4,CV_64FC1);
    R = rotate(theta);
    T = translate(x,y);
    A = T*R;


    return A;
  }


void laserlocalToglobal(double x, double y, double heading, double& xgrid, double& ygrid){


  static cv::Mat laser_seg_coord(4,1,CV_64FC1);
  static cv::Mat laser_seg_global_coord(4,1,CV_64FC1);


  laser_seg_coord.at<double>(0,0) = x;
  laser_seg_coord.at<double>(1,0) = y;
  laser_seg_coord.at<double>(2,0) = 0;
  laser_seg_coord.at<double>(3,0) = 1;

  laser_seg_global_coord = convmat(gps_pt_.x(),gps_pt_.y(),heading-M_PI/2) * laser_seg_coord;


  xgrid = laser_seg_global_coord.at<double>(0,0);
  ygrid = laser_seg_global_coord.at<double>(1,0);
}

  void LidarObjectCallback(const kut_ugv_sensor_fusion::lidar_object_listConstPtr& msg){

    int object_num = msg->object.size();
    int laser_idxs;
    double object_width;
    double object_height;
    double object_center_x;
    double object_center_y;
    double object_center_x_global;
    double object_center_y_global;
    double laser_distance;
    double laser_x;
    double laser_y;
    double global_laser_x;
    double global_laser_y;
    double object_lat_offset;
    double narrow_lat_offset;
    int object_label;
    static int old_object_label;
    static int tracking_count;

    int max_x_idx;

    narrow_path_flag_ = 0;

    for (int i=0; i<object_num; i++){
      laser_idxs = msg->object[i].idxs.size();
      object_width = msg->object[i].width;
      object_height = msg->object[i].height;

      object_center_y = msg->object[i].centroid.x;
      object_center_x = -1*(msg->object[i].centroid.y);
      object_label = msg->object[i].label;

      laserlocalToglobal(object_center_x,object_center_y,current_heading_,object_center_x_global,object_center_y_global);


      object_lat_offset = lateraloffset2(object_center_x_global,object_center_y_global);

      if (abs(object_lat_offset) < 7){
        if (object_width > 6 || object_height > 6){

          if(object_label == old_object_label){
            tracking_count++;
          }
          old_object_label = object_label;

          if(tracking_count >20){
            ROS_INFO("Narrow path wall");
            tracking_count=0;
            max_x_idx = msg->object[i].max_x_idx;

            laser_x = laser_x_[max_x_idx];
            laser_y = laser_y_[max_x_idx];



            laserlocalToglobal(laser_x,laser_y,current_heading_,global_laser_x,global_laser_y);
            narrow_path_start_.setValue(global_laser_x,global_laser_y,0);
            //narrow_lat_offset = lateraloffset2(global_laser_x,global_laser_y);
            narrow_path_flag_ = 1;
          }


          //narrow_path_flag_ = 1;

        }

        else{
          narrow_path_flag_ = 0;
        }
      }
    }




    narrow_path_flag_ = 0;
    //ROS_INFO("object_num: %d",object_num);
  }


  void VelocityLimitCallback(const kut_ugv_msgs::VelocityLimitConstPtr& msg){

    velocity_x_ = msg ->x;
    velocity_y_ = msg ->y;
    velocity_max_ = msg ->max_velocity;
    action_type_ = msg ->type;
    ROS_INFO("message_get");

  }

  void RoofLaserCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
    roof_cloud_.clear();
    sensor_msgs::PointCloud2 in_msg = *msg;
    pcl::fromROSMsg(in_msg,roof_cloud_) ;

  }
  void BumperLaserCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
    bumper_cloud_.clear();
    sensor_msgs::PointCloud2 in_msg = *msg;
    pcl::fromROSMsg(in_msg,bumper_cloud_) ;

  }


  void VehicleStateCallback(const kut_ugv_vehicle::StateStampedConstPtr& msg){
    crio_speed_ = msg->state.velocity;
    theta_ = msg ->state.theta;
    beta_ = msg->state.beta;


    //ROS_INFO("speed: %f, theta: %f, beta: %f",crio_speed_, theta_, beta_);
    }

  void GPSCallback(const nav_msgs::OdometryConstPtr& msg)
  {

    double sec = msg->header.stamp.sec;
    double nsec = msg->header.stamp.nsec;
    static double old_sec;
    static double old_nsec;
    static int loop_count=0;

    static tf::Point gps_old_pt = gps_pt_;

    static double heading_data[data_size];
    static double gps_x[data_size];
    static double gps_y[data_size];
    static double heading;

    double lookahead = 3.0;



    gps_pt_.setValue(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0);

    tf::Pose pose;
    tf::poseMsgToTF(msg->pose.pose, pose);

    double tmp, Yaw;
    pose.getBasis().getRPY(tmp, tmp, Yaw);

    //// Current Heading ////

      heading = Yaw;
      current_heading_ = heading;

    static int flag_=0;
    if (flag_ < 1){
       init_gpsx = gps_pt_.x();
       init_gpsy = gps_pt_.y();
       flag_+=1;
     }

    double new_gpsx = gps_pt_.x() - init_gpsx;
    double new_gpsy = gps_pt_.y() - init_gpsy;

    int grid_x;
    int grid_y;
    int grid_gpsx;
    int grid_gpsy;

    int refresh_x;
    int refresh_y;

    dotTogrid(new_gpsx,new_gpsy,grid_gpsx,grid_gpsy);

    for (int i=0;i<local_mapsize;i++){
      for (int j=0;j<local_mapsize;j++){
        refresh_x = grid_gpsx - (local_mapsize/2) + i;
        refresh_y = grid_gpsy - (local_mapsize/2) + j;
        if (refresh_x > mapsize-1 || refresh_y > mapsize-1 ){
          continue;
        }
        else if (refresh_x < 1 || refresh_y < 1){
          continue;
        }
        global_map.at<unsigned char>(refresh_x,refresh_y) = 120;
      }
    }

    // convert laser data to global coordinate //

    static cv::Mat laser_coord(4,1141,CV_64FC1);
    static cv::Mat laser_global_coord(4,1141,CV_64FC1);

    double laser_gpsx;
    double laser_gpsy;

    for (int i=0;i<1141;i++){
      laser_coord.at<double>(0,i) = laser_x_[i];
      laser_coord.at<double>(1,i) = laser_y_[i];
      laser_coord.at<double>(2,i) = 0;
      laser_coord.at<double>(3,i) = 1;
    }
    laser_global_coord = convmat(new_gpsx,new_gpsy,heading-M_PI/2) * laser_coord;

    for (int i=0;i<1141;i++){
      if (sqrt(pow(laser_x_[i],2) + pow(laser_y_[i],2)) > 50 ){
        continue;
      }
      laser_gpsx = laser_global_coord.at<double>(0,i);
      laser_gpsy = laser_global_coord.at<double>(1,i);
      dotTogrid(laser_gpsx,laser_gpsy,grid_x,grid_y);
      if (grid_x > mapsize-1 || grid_y > mapsize-1 ){
        continue;
      }
      else if (grid_x < 1 || grid_y < 1){
        continue;
      }
      global_map.at<unsigned char>(grid_x,grid_y) = 240;
    }



    int roof_cloud_grid_x;
    int roof_cloud_grid_y;
    double roof_cloud_x;
    double roof_cloud_y;

    int bumper_cloud_grid_x;
    int bumper_cloud_grid_y;
    double bumper_cloud_x;
    double bumper_cloud_y;


    //static cv::Mat cloud_coord(4,cloud_.points.size(),CV_64FC1);
    //static cv::Mat cloud_global_coord(4,cloud_.points.size(),CV_64FC1);

    /*
    for (size_t j = 0; j < cloud_.points.size(); j++){
      cloud_coord.at<double>(0,j) = cloud_.points[j].y;
      cloud_coord.at<double>(1,j) = cloud_.points[j].x;
      cloud_coord.at<double>(2,j) = 0;
      cloud_coord.at<double>(3,j) = 1;
    }
    cloud_global_coord = convmat(0,0,heading-M_PI/2) * cloud_coord;
    */



    // Cloud laser data //
/*
    for (size_t j = 0; j < roof_cloud_.points.size(); j++){

      //cloud_x = cloud_global_coord.at<double>(0,j);
      //cloud_y = cloud_global_coord.at<double>(1,j);

      //dotTogrid(cloud_x, cloud_y, cloud_grid_x, cloud_grid_y);

      dotTogrid(roof_cloud_.points[j].x - init_gpsx, roof_cloud_.points[j].y - init_gpsy, roof_cloud_grid_x, roof_cloud_grid_y);

      if (roof_cloud_grid_x > mapsize-10 || roof_cloud_grid_y > mapsize-10 ){
        continue;
      }
      if (roof_cloud_grid_x < 10 || roof_cloud_grid_y < 10){
        continue;
      }
      global_map.at<unsigned char>(roof_cloud_grid_x,roof_cloud_grid_y) = 240;
          //ROS_INFO("cloud :%d", cloud_grid_x);
    }
*/

/*
    for (size_t j = 0; j < bumper_cloud_.points.size(); j++){

      dotTogrid(bumper_cloud_.points[j].x - init_gpsx, bumper_cloud_.points[j].y - init_gpsy, bumper_cloud_grid_x, bumper_cloud_grid_y);

      if (bumper_cloud_grid_x > mapsize-10 || bumper_cloud_grid_y > mapsize-10 ){
        continue;
      }
      if (bumper_cloud_grid_x < 10 || bumper_cloud_grid_y < 10){
        continue;
      }
      global_map.at<unsigned char>(bumper_cloud_grid_x,bumper_cloud_grid_y) = 240;
    }
*/






      lookahead_x = gps_pt_.x() + lookahead*cos(heading);
      lookahead_y = gps_pt_.y() + lookahead*sin(heading);


      //// Closest Way Point ////

      int current;
      int lookahead_current;
      static int old_current;

      current = ClosetWaypoint_order(old_current, gps_pt_.x(), gps_pt_.y(), current_heading_);
      lookahead_current = ClosetWaypoint_order(old_current, lookahead_x, lookahead_y, current_heading_);


      old_current = current;

      ////////------------ Planning Simulation ------------////////

      static int path_choice = (num_path-1)/2;
      static double shifting[num_path];
      double path_width = 1.5/2;

      static double trajectory_dist;
      static int collision_flag;
      static double maximum_curvature;

      static double traj_dist[num_path];
      static int col_flag[num_path];
      static double max_curve[num_path];


      for (int i=0;i<num_path;i++){
         shifting[i] = -path_width*((num_path-1)/2) + i * path_width;
       }


      //if (loop_count%1 == 0){

      //double velocity = 15.0; //  km/h
      double velocity = speed_; //km/h
      velocity = velocity * 1000/3600; // m/s

      velocity = crio_speed_;

      double min_lat = 20.0;
      path_choice = (num_path-1)/2;


      //  Traffic Arrow Motion //

      double traffic_arrow_latoff;
      int traffic_arrow_current = 0;

      action_type_ = 0;

      if (action_type_ != 0){ // To shift the reference path to the center of traffic light;
        //traffic_arrow_latoff = lateraloffset2(velocity_x_,velocity_y_);
        //traffic_arrow_current = object_closest(velocity_x_,velocity_y_);
        traffic_arrow_current = current+1;
        traffic_arrow_latoff = 0.0;
        for(int i=0;i<num_path;i++){
          shifting[i] += traffic_arrow_latoff;
        }
      }


      //  Narrow Path Motion //

      tf::Point narrow_start_local;
      tf::Point narrow_start_global;

      int length_idxs;
      for (int i=current; i<wp_.size();i++){
        for (int j=0;j<length_idxs;j++){
          narrow_start_global.distance(wp_[i]);
          narrow_path_start_.distance(wp_[i]);
        }
      }

      double narrow_lat_offset=0.0;
      double narrow_dist = 0.0;
      double narrow_path_dist = 105.0;
      int narrow_start_current = 0;
      int narrow_iter = 0;
      static int narrow_goal = 0;


      if (narrow_path_flag_ == 1){

        //ROS_INFO("narrow_lat_offset: %f", narrow_lat_offset);

        narrow_lat_offset = lateraloffset2(narrow_path_start_.x(),narrow_path_start_.y());
        narrow_start_current = object_closest(narrow_path_start_.x(),narrow_path_start_.y());
        while( narrow_dist < narrow_path_dist){
          narrow_dist += wp_[narrow_start_current+narrow_iter].distance(wp_[narrow_start_current+narrow_iter+1]);
          narrow_iter += 1;
        }
        narrow_goal = narrow_start_current + narrow_iter;
      }


      if (narrow_goal != 0 && current < narrow_goal){
        for(int i=0;i<num_path;i++){
          shifting[i] += narrow_lat_offset;
        }


      }

      //ROS_INFO("narrow_lat_offset: %f", narrow_lat_offset);


      for (int i=0;i<num_path;i++){
        trajectory_simulation(i,current, gps_pt_.x(), gps_pt_.y(), current_heading_, shifting[i], velocity, trajectory_dist,collision_flag, maximum_curvature);
        //trajectory_simulation(i,lookahead_current, lookahead_x, lookahead_y, current_heading_, shifting[i], velocity, trajectory_dist,collision_flag, maximum_curvature);
        traj_dist[i] = trajectory_dist;
        col_flag[i] = collision_flag;
        max_curve[i] = maximum_curvature;
      }





      double path_cost[num_path];
      double w_lat = 0.05;
      double w_curv = 0;//1;
      double min_cost = DBL_MAX;
      double max_dist = DBL_MIN;
      double collision_count =0;
      static int old_choice = (num_path-1)/2;
      int path_choice_buf = (num_path-1)/2;


      for (int i=0;i<num_path;i++){
        path_cost[i] = w_lat * abs(shifting[i]) + w_curv * max_curve[i];
        if (col_flag[i] == 0){
//          if (abs(shifting[i] < min_lat)){
//            min_lat = abs(shifting[i]);
//            path_choice = i;
//          }
          if (path_cost[i] < min_cost){
            min_cost = path_cost[i];
            path_choice_buf = i;
          }
        }
        else{
          collision_count += 1;
        }
      }

      //path_choice = path_choice_buf;
      //ROS_INFO("lateral: %f, curvature: %f",shifting[path_choice_buf], max_curve[path_choice_buf]);


      if (abs(traj_dist[path_choice_buf] - traj_dist[old_choice])/traj_dist[old_choice] < 0.12 )
      {
        path_choice_buf = old_choice;
      }

//      static int path_select_count = 0;
//
//      if (path_choice_buf == old_choice){
//        path_select_count++;
//      }
//      else{
//        path_select_count = 0;
//      }
//
//      if (path_select_count > 5){
//        path_choice = path_choice_buf;
//      }
//      else{
//        path_choice = old_choice;
//      }

      path_choice = path_choice_buf;
      //old_choice = path_choice;


      if (collision_count == 0 ){
        path_choice = (num_path-1)/2;
      }
      else{
        if (col_flag[int(num_path-1)/2] == 0 && collision_count < 9){
          path_choice = (num_path-1)/2;
        }
        else{
          if (col_flag[old_choice] == 0){
            path_choice = old_choice;
          }
          else if (collision_count == num_path){
            for (int i=0;i<num_path;i++){
              if (traj_dist[i]>max_dist){
                max_dist = traj_dist[i];
                path_choice = i;
                //if (abs(traj_dist[i] - traj_dist[old_choice])/traj_dist[old_choice] < 0.12 ){
                //  path_choice = old_choice;
                //}

              }
            }
          }
          else{
            path_choice = path_choice_buf;
          }
        }
      }

  old_choice = path_choice;

      //----  Traffic Light Arrow  -----//

      if ( current < traffic_arrow_current && action_type_ != 0){

        if (action_type_ == 1){  // Left arrow traffic light
          min_cost = DBL_MAX;
          for (int i = (num_path+1)/2+2;i<num_path;i++){
            path_cost[i] = w_lat * abs(shifting[i]) + w_curv * max_curve[i];
            if (col_flag[i] == 0){
              if (path_cost[i] < min_cost){
                min_cost = path_cost[i];
                path_choice = i;
              }
            }
          }
        }
        else if (action_type_ == 2){ // Right arrow traffic light
          min_cost = DBL_MAX;
          for (int i = 0;i<(num_path-1)/2-1;i++){
            path_cost[i] = w_lat * abs(shifting[i]) + w_curv * max_curve[i];
            if (col_flag[i] == 0){
              if (path_cost[i] < min_cost){
                min_cost = path_cost[i];
                path_choice = i;
              }
            }
          }
        }
      }


      //ROS_INFO("%d %d %d %d %d %d %d",col_flag[0],col_flag[1],col_flag[2],col_flag[3],col_flag[4],col_flag[5],col_flag[6]);

      //if ((traj_dist[old_choice] - traj_dist[path_choice]) < traj_dist[old_choice]*0.1){
      //  path_choice = old_choice;
      //}

      int t_lookahead_point;

      t_lookahead_point = T_closestwaypoint(path_choice, lookahead_x, lookahead_y, current_heading_);


      //path_choice = 3; // no path planning

      if (collision_count >= 12){
        planner_speed_limit = 10;
      }
      else if (collision_count >= 10){
        planner_speed_limit = 13;
      }
      else if (collision_count >= 8){
        planner_speed_limit = 18;
      }
      else if (collision_count >= 6){
        planner_speed_limit = 25;
      }
      else if (collision_count >= 4){
        planner_speed_limit = 30;
      }
      else if (collision_count >= 2){
        planner_speed_limit = 35;
      }
      else if (collision_count >= 1){
        planner_speed_limit = 41;
      }
      else{
        planner_speed_limit = 41;
      }

      planner_speed_limit_x = trajectory_[path_choice][planning_sample-1][1];
      planner_speed_limit_y = trajectory_[path_choice][planning_sample-1][2];



      path_.clear();

      for (int i=0;i<planning_sample;i++){
        tf::Point path_pt;
        path_pt.setValue(trajectory_[path_choice][i][1],trajectory_[path_choice][i][2],0.0);
        path_.push_back(path_pt);
      }


      // way points visualize //

      int wp_length = wp_.size();
      double new_wpx;
      double new_wpy;
      int grid_wpx[wp_length];
      int grid_wpy[wp_length];
      for (int j = 0; j < wp_length; j++){
        new_wpx = wp_[j].x() - init_gpsx;
        new_wpy = wp_[j].y() - init_gpsy;
        dotTogrid(new_wpx,new_wpy,grid_wpx[j],grid_wpy[j]);
        if (grid_wpx[j] > mapsize-1 || grid_wpy[j] > mapsize-1 ){
          continue;
        }
        else if (grid_wpx[j] < 1 || grid_wpy[j] < 1){
          continue;
        }
        global_map.at<unsigned char>(grid_wpx[j],grid_wpy[j]) = 150;
      }





      //// ---- Reference path collision check ---- ////

      double wp_dist = 0.0;
      double slope;
      int bound_x;
      int bound_y;
      int wp_i = current;

      while ( wp_dist < 2){

        //slope = -atan2(grid_wpx[wp_i+1]-grid_wpx[wp_i], grid_wpy[wp_i+1]-grid_wpy[wp_i]);
        if (wp_[wp_i+1].y() != wp_[wp_i].y()){
          slope = -atan2(wp_[wp_i+1].x()-wp_[wp_i].x(), wp_[wp_i+1].y()-wp_[wp_i].y());
        }

        for( int i=-10;i<10;i++){
          bound_x = int(grid_wpx[wp_i] +i*cos(slope));
          bound_y = int(grid_wpy[wp_i] +i*sin(slope));
          if (bound_x > mapsize-1 || bound_y > mapsize-1){
            continue;
          }
          if (bound_x < 1 || bound_y < 1){
            continue;
          }
          //global_map.at<unsigned char>(bound_x,bound_y) = 150;
        }
        wp_dist += wp_[wp_i].distance(wp_[wp_i+1]);
        wp_i += 1;
      }


      //ROS_INFO("size: %d , path_x: %f, path_y: %f",path_.size(), path_[0].x(),path_[0].y());





    cv::Mat array_path(4,50,CV_64FC1);

    double new_array_path_x1;
    double new_array_path_y1;
    int grid_array_path_x1;
    int grid_array_path_y1;
    double new_array_path_x2;
    double new_array_path_y2;
    int grid_array_path_x2;
    int grid_array_path_y2;

    //double slope;
    //int bound_x;
    //int bound_y;

    for (int i=0;i<num_path;i++){
      //if (i != path_choise) continue;
      for (int j=0;j<planning_sample-1;j++){
        new_array_path_x1 = trajectory_[i][j][1] - init_gpsx;
        new_array_path_y1 = trajectory_[i][j][2] - init_gpsy;
        new_array_path_x2 = trajectory_[i][j+1][1] - init_gpsx;
        new_array_path_y2 = trajectory_[i][j+1][2] - init_gpsy;

        dotTogrid(new_array_path_x1,new_array_path_y1, grid_array_path_x1,grid_array_path_y1);
        dotTogrid(new_array_path_x2,new_array_path_y2, grid_array_path_x2,grid_array_path_y2);
        if (grid_array_path_x2 > mapsize-1 || grid_array_path_y2 > mapsize-1 ){
          continue;
        }
        else if (grid_array_path_x2 < 1 || grid_array_path_y2 < 1){
          continue;
        }

        if (grid_array_path_x1==grid_array_path_x2 && grid_array_path_y1==grid_array_path_y2 ){
          continue;
        }

        if ( i == path_choice){
          cv::line(global_map,cv::Point(grid_array_path_y1,grid_array_path_x1),cv::Point(grid_array_path_y2,grid_array_path_x2),0);
        }
        else{
          cv::line(global_map,cv::Point(grid_array_path_y1,grid_array_path_x1),cv::Point(grid_array_path_y2,grid_array_path_x2),100);
        }
      }
    }

    //ROS_INFO("path: %d, moving_dist: %f, collision check: %d, max_curvature: %f",path_choice, traj_dist[path_choice], col_flag[path_choice], max_curve[path_choice]);
    //ROS_INFO("path: %d, collision check: %d, lateral offset: %f",path_choice, col_flag[path_choice], lateral_offset_);


    //ROS_INFO("error test 1");


    cv::Mat small_map(global_map, cv::Rect(grid_gpsy - local_mapsize/2, grid_gpsx - local_mapsize/2, local_mapsize, local_mapsize));
    small_map.copyTo(grid_map_);
    cv::imshow("Occupancy grid Map",small_map);
    cv::waitKey(2);


    if (sec == old_sec){
      sample_time = (nsec - old_nsec)/pow(10,9);
    }
    else{
      sample_time = (nsec+pow(10,9) - old_nsec)/pow(10,9);
    }
    //ROS_INFO("sec: %f, nsec: %f", sec, nsec);
    //ROS_INFO("sample time: %f", sample_time);




    gps_old_pt = gps_pt_;
    old_sec = sec;
    old_nsec = nsec;

    loop_count+=1;

  }

  void LaserCallback(const sensor_msgs::LaserScanConstPtr& msg)
  {

    laser_sec = msg->header.stamp.sec;
    laser_nsec = msg->header.stamp.nsec;


    std::vector<float> data = msg->ranges;
    std::vector<double>::iterator it;

    double angle_resolution = msg->angle_increment;
    double laser_angle = msg->angle_min + M_PI / 2;

    double laser_x[data.size()];
    double laser_y[data.size()];
    double x;
    double y;


    static double r_path_angle_data[data_size];

    static tf::Point gps_old;



    //heading = current_heading_;


    for (size_t i = 0; i < data.size(); ++i)
    {
      laser_data_[i] = data[i];
      if (data[i] > 0.0 && data[i] < 70.0)
      {
        laser_x[i] = data[i] * cos(laser_angle);
        laser_y[i] = data[i] * sin(laser_angle);
      }
      else
      {
        laser_x[i] = 70.0 * cos(laser_angle);
        laser_y[i] = 70.0 * sin(laser_angle);
      }

      laser_x_[i] = laser_x[i];
      laser_y_[i] = laser_y[i] + GPStoLaser;


      laser_angle += angle_resolution;
      //log_file_ << laser_x_[i] <<" "<< laser_y_[i] <<" "<<gps_pt_.x()<<" "<<gps_pt_.y()<<" "<<heading<<"\n";
      //log_file_ <<gps_pt_.x()<<" "<<gps_pt_.y()<<"\n";
    }


  }


  void A_star(int current, double gpsx, double gpsy){
    std::vector<tf::Point> start;
    std::vector<tf::Point> goal;




  }



  int T_closestwaypoint (int path, double gpsx, double gpsy, double heading){

    int order;
    double dist;
    double min_dist = DBL_MAX;

    for (int i=0; i<planning_sample;i++)
    {
      dist = sqrt(pow((gpsx - trajectory_[path][i][1]),2) + pow((gpsy - trajectory_[path][i][2]),2));
      if (min_dist > dist)
      {
        min_dist = dist;
        order = i;
      }
    }
    return order;
  }

  int closestwaypoint2 (int old_current, double gpsx, double gpsy, double heading){
    int order;
    double dist;
    double min_dist = DBL_MAX;
    int over_order;

    int length = wp_.size();

    if (old_current < 2){
      old_current = 2;
    }

    for (int i=old_current-2; i<old_current+100;i++)
    {
      if (i > length-1){
        over_order = i-length;
        dist = sqrt(pow(gpsx-wp_[over_order].x(),2) + pow(gpsy-wp_[over_order].y(),2));
      }
      else{
      dist = sqrt(pow(gpsx-wp_[i].x(),2) + pow(gpsy-wp_[i].y(),2));
      }
      if (min_dist > dist)
      {
        min_dist = dist;
        order = i;
        if(order > length-1){
          order = i-length;
        }
      }

    }
    return order;
  }

  int ClosetWaypoint_order(int old_current,double gpsx, double gpsy, double heading){

    tf::Point vehicle_front;
    tf::Point gps;
    double dist_gpsTolaser = 3.66;
    double vehicle_front_x;
    double vehicle_front_y;

    gps.setX(gpsx);
    gps.setY(gpsy);

    vehicle_front_x = gpsx + dist_gpsTolaser * cos(heading);
    vehicle_front_y = gpsy + dist_gpsTolaser * sin(heading);
    vehicle_front.setX(vehicle_front_x);
    vehicle_front.setY(vehicle_front_y);

    double dist = 0.0;
    double min_dist = DBL_MAX;
    static size_t order = 0;

    double wpx;
    double wpy;

    for (size_t j = 0; j < wp_.size(); j++)
    {

      dist = gps.distance(wp_[j]);

      if (min_dist > dist)
      {
        min_dist = dist;
        order = j;
      }
    }

    return order;
  }

  int closestwaypoint_origin(double pose_x, double pose_y){

      tf::Point position;

      position.setX(pose_x);
      position.setY(pose_y);


      double dist = 0.0;
      double min_dist = DBL_MAX;
      static size_t order = 0;


      for (size_t j = 0; j < wp_.size(); j++)
      {
        dist = position.distance(wp_[j]);
        if (min_dist > dist)
        {
          min_dist = dist;
          order = j;
        }
      }
      if (order > wp_.size())
      {
        order = wp_.size();
      }

      return order;
    }

  int object_closest(double gpsx, double gpsy){

    tf::Point gps;

    gps.setX(gpsx);
    gps.setY(gpsy);

    double dist = 0.0;
    double min_dist = DBL_MAX;
    static size_t order = 0;

    double wpx;
    double wpy;

    for (size_t j = 0; j < wp_.size(); j++)
    {
      dist = gps.distance(wp_[j]);
      if (min_dist > dist)
      {
        min_dist = dist;
        order = j;
      }
    }
    return order;
  }



  void trajectory_simulation( int index,int current, double pose_x, double pose_y, double heading, double shifting, double velocity, double& moving_dist, int& collision_flag, double& max_curve){

    double angle_diff;
    double lat_off;
    double old_angle_diff;
    double old_lat_off;
    double angle_diff_d;
    double lat_off_d;
    double curv;
    double theta;
    double theta_dot;

    double pose_x_dot;
    double pose_y_dot;
    double pose_x_old;
    double pose_y_old;
    double steering;

    double planning_dist;

    double slope;

    double R;
    double beta;
    double Cx;
    double Cy;
    double delta_dist;

    double offset = 0.0;
    //double lateral_acc = abs(shifting)/lat_off_rate;
    double lateral_acc = 0.1;

    theta = heading;



    if (velocity*3600/1000 > 40.0){
      planning_dist = 40.0;
    }

    else if (velocity*3600/1000 > 10.0){
      planning_dist = (40.0-20.0)/(40.0-10.0) * (velocity*3600/1000) + 40/3;
    }
    else{
      planning_dist = 2 * (velocity*3600/1000);
      //planning_dist = 20;
    }
    planning_dist += 4.0;

    //ROS_INFO("look_ahead : %f", look_ahead);
    if (planning_dist > 40){
      planning_dist = 40;
    }
    //planning_dist = 20;

    delta_dist = planning_dist/planning_sample;

    if (velocity < 1.39){   // 5 km/h
      velocity = 1.39;
    }


    int order = 0;

    int collision_bound_x;
    int collision_bound_y;
    //while(moving_dist < look_ahead){


    collision_flag = 0;
    moving_dist = 0;

    double x_d;
    double y_d;
    double x_dd;
    double y_dd;

    double look_x;
    double look_y;

    int old_current = current;
    int lookahead_current = current;
    int old_lookahead = current;

    for (int order=0;order<planning_sample;order++){

      if (collision_flag != 1 && moving_dist < planning_dist){

        //// Predict with Vehicle Kinematics ////

        look_x = pose_x + 2.7*cos(theta);
        look_y = pose_y + 2.7*sin(theta);

        //lookahead_current =closestwaypoint2(old_lookahead,look_x,look_y,theta);
        //current =closestwaypoint2(old_current,look_x,look_y,theta);
        lookahead_current =closestwaypoint2(old_lookahead,pose_x,pose_y,theta);
        //current = ClosetWaypoint_order(old_current,pose_x,pose_y,theta); // closest way point

        angle_diff = angle_difference(lookahead_current, theta);
        if (angle_diff> M_PI){
          angle_diff = angle_diff - 2*M_PI;
        }
        else if (angle_diff< -M_PI){
          angle_diff = angle_diff + 2*M_PI;
        }

        if(lookahead_current >= (wp_.size()-2) ){
          lat_off = lateraloffset(look_x, look_y, wp_[lookahead_current].x(), wp_[lookahead_current].y(), wp_[0].x(), wp_[0].y());
          //ROS_INFO("test");
        }
        else{
          lat_off = lateraloffset(look_x, look_y, wp_[lookahead_current].x(), wp_[lookahead_current].y(), wp_[lookahead_current+1].x(), wp_[lookahead_current+1].y());
        }
        //ROS_INFO("%d %d",wp_.size(),lookahead_current);
        //ROS_INFO("wp_x: %f",wp_[4599].x());


        lat_off = lat_off + 0.0;

        if (shifting > 0){
          offset += lateral_acc;
          if (offset > shifting){
            offset = shifting;
          }
        }

        if (shifting < 0){
          offset -= lateral_acc;
          if (offset < shifting){
            offset = shifting;
          }
        }

        lat_off += shifting;
        //lat_off += offset;
        curv = curvature[lookahead_current].x();

        steering = steering_control(angle_diff, 0.0, lat_off, 0.0, curv, velocity);

        beta = delta_dist/wheel_base*tan(steering);

        //ROS_INFO("angle_diff: %f, lat_offset: %f",angle_diff, lat_off);
        //ROS_INFO("steering: %f, beta: %f, pose_x_dot: %f, pose_y_dot: %f", steering, beta, pose_x_dot, pose_y_dot);

        if (abs(beta)<0.001){
          pose_x_dot = delta_dist * cos(theta);
          pose_y_dot = delta_dist * sin(theta);

          theta += beta;
          pose_x += pose_x_dot;
          pose_y += pose_y_dot;
          //ROS_INFO("A");
        }
        else{

          R = wheel_base/beta;
          Cx = pose_x - delta_dist * sin(theta);
          Cy = pose_y + delta_dist * cos(theta);

          theta += beta;
          pose_x = Cx + sin(theta)*R;
          pose_y = Cy - cos(theta)*R;
          //ROS_INFO("B");
        }
        moving_dist += delta_dist;


        //// collision check ////

       int grid_pose_x1;
       int grid_pose_y1;
       int grid_pose_x2;
       int grid_pose_y2;

       int collision_range;


       if (velocity*3600/1000 > 30.0){
         collision_range = 11.0;
       }
       else if (velocity*3600/1000 > 20.0){
         collision_range = 10.0;
       }
       else if (velocity*3600/1000 > 15.0){
         collision_range = 9.0;
       }
       else if (velocity*3600/1000 > 10.0){
         collision_range = 8.0;
       }


       dotTogrid(pose_x_old - init_gpsx,pose_y_old - init_gpsy, grid_pose_x1,grid_pose_y1);
       dotTogrid(pose_x - init_gpsx,pose_y - init_gpsy, grid_pose_x2,grid_pose_y2);

       //if (grid_pose_y2 != grid_pose_y1 && pose_x_old != 0){
       if (pose_x_old != 0){
         //slope = atan2(-(grid_pose_x2-grid_pose_x1),grid_pose_y2-grid_pose_y1);
         slope = atan2(-(pose_x-pose_x_old),pose_y-pose_y_old);

         for (int i=-collision_range;i<collision_range+1;i++){
           collision_bound_x = int(grid_pose_x2 + i*cos(slope));
           collision_bound_y = int(grid_pose_y2 + i*sin(slope));
           if ( collision_bound_x > mapsize-1 || collision_bound_y > mapsize-1 ){
             continue;
           }
           else if ( collision_bound_x < 1 || collision_bound_y < 1 ){
             continue;
           }
           //global_map.at<unsigned char>(collision_bound_x,collision_bound_y) = 200;
           if (global_map.at<unsigned char>(collision_bound_x,collision_bound_y) == 240){
             collision_flag = 1;
           }
         }
       }

       trajectory_[index][order][1] = pose_x;
       trajectory_[index][order][2] = pose_y;
       //ROS_INFO("test1");
      }
      else{
        if (order > 0){
          trajectory_[index][order][1] = trajectory_[index][order-1][1];
          trajectory_[index][order][2] = trajectory_[index][order-1][2];
        }
        //ROS_INFO("test2");
      }

      pose_x_old = pose_x;
      pose_y_old = pose_y;
      //order += 1;

      old_angle_diff = angle_diff;
      old_lat_off = lat_off;
      old_current = current;
      old_lookahead = lookahead_current;
    }
    // curvature //

//    double curvature;
//    max_curve = DBL_MIN;
//    for (int i=2;i<planning_sample;i++){
//      x_d = trajectory_[index][i][1] - trajectory_[index][i-1][1];
//      y_d = trajectory_[index][i][2] - trajectory_[index][i-1][2];
//      x_dd = trajectory_[index][i][1] - 2*trajectory_[index][i-1][1] + trajectory_[index][i-2][1];
//      y_dd = trajectory_[index][i][2] - 2*trajectory_[index][i-1][2] + trajectory_[index][i-2][2];
//      if (x_d != 0 && x_dd !=0){
//        curvature = calc_curvature(x_d, y_d, x_dd, y_dd);
//        if (curvature < 0){
//          curvature = -curvature;
//        }
//      }
//      if (curvature > max_curve){
//        max_curve = curvature;
//      }
//    }
    max_curve = 0;
    //ROS_INFO("xd: %f, yd: %f, xdd: %f, ydd: %f",x_d, y_d, x_dd, y_dd);
    //ROS_INFO("curvature: %f",max_curve);

  }

  double T_angle_difference(int path, int current, double heading){

    double r_path_angle;
    double angle_diff;

    if (current >= wp_.size()){
      r_path_angle = atan2(trajectory_[path][0][2] - trajectory_[path][current][2], trajectory_[path][0][1] - trajectory_[path][current][1]);
    }
    else{
      r_path_angle = atan2(trajectory_[path][current+1][2] - trajectory_[path][current][2], trajectory_[path][current+1][1] - trajectory_[path][current][1]);
    }
    angle_diff = r_path_angle - heading;

    return angle_diff;
  }

  double angle_difference(int current, double heading){

    double r_path_angle;
    double angle_diff;

    if (current >= (wp_.size()-2)){
      r_path_angle = atan2(wp_[0].y() - wp_[current].y(), wp_[0].x() - wp_[current].x());
    }
    else{
      r_path_angle = atan2(wp_[current + 1].y() - wp_[current].y(), wp_[current + 1].x() - wp_[current].x());
    }
    angle_diff = r_path_angle - heading;

    return angle_diff;
  }


  double lateraloffset(double cur_x,double cur_y,double x1,double y1,double x2,double y2){

    double lat_off;

    lat_off = ((cur_x-x1)*(y2-y1)-(cur_y-y1)*(x2-x1))/(sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)));


    return lat_off;
  }


  double lateraloffset2(double cur_x,double cur_y){

    double lat_off;

    double x1;
    double y1;
    double x2;
    double y2;

    int order;

    order = closestwaypoint_origin(cur_x,cur_y);
    x1 = wp_[order].x();
    y1 = wp_[order].y();

    if (order >= wp_.size()-1){
      x2 = wp_[0].x();
      y2 = wp_[0].y();
    }
    else{
      x2 = wp_[order+1].x();
      y2 = wp_[order+1].y();
    }
    lat_off = ((cur_x-x1)*(y2-y1)-(cur_y-y1)*(x2-x1))/(sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)));

    return lat_off;
  }

  double calc_curvature(double xd, double yd, double xdd, double ydd){

    double curv;
    double den;

    den = pow(xd*xd + yd*yd,1.5);
    if (den != 0){
      curv = (xd*ydd - yd*xdd)/den;
//      if (abs(curv) > 1){
//        curv = 0;
//      }
    }
    return curv;
  }


  double steering_control(double angle_diff, double angle_diff_d, double lat_off, double lat_off_d, double curvature, double velocity){

    double steering_angle;
    double kp_angle;
    double kp_lat;
    double kp_curv;
    double kd_angle;
    double kd_lat;

    double angle_control;
    double lat_control;
    double curv_control;

    kp_angle = 0.935;
    kd_angle = 0.5;
    kp_lat = 0.3;//1;
    kd_lat = 0.1;
    kp_curv = 2.7;

    angle_control = kp_angle * angle_diff + kd_angle * angle_diff_d;
    lat_control = atan2(kp_lat * lat_off + kd_lat * lat_off_d, velocity);
    curv_control = kp_curv*curvature;

    steering_angle = angle_control + lat_control + curv_control;

    return steering_angle;
  }

  // Load parameters etc
  int init()
  {
    node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));
    pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

    pnode_->param("publish_rate", publish_rate_, 100);

    //pnode_->param("crio_node_ns", crio_node_ns_, "/crio_comm");
    //node_->param(crio_node_ns_ + "/publish_rate", publish_rate_, 100);
    //if( !node_->GetParam(....) )
    //    ROS_FATAL("Can't find cRIO node publish_rate parameter!");

    pnode_->param("waypoint_file", wp_filename_, std::string(""));
    pnode_->param("curvature_file", curvature_filename_, std::string(""));
    pnode_->param("velocity_file", velocity_filename_, std::string(""));
    pnode_->param("log_file", log_filename_, std::string(""));
    pnode_->param("log_file2", log_filename2_, std::string(""));

    heading_file_.open(log_filename_.c_str());
    log_file_.open(log_filename2_.c_str());

    if(!heading_file_.is_open())
    {
      ROS_FATAL_STREAM("Unable to open log file: " << log_filename_);
      return -1;
    }

    if (!wp_filename_.size())
    {
      ROS_FATAL("Parameter waypoint_file is not specified or empty!");
      return -1;
    }
    if (!curvature_filename_.size())
    {
      ROS_FATAL("Parameter curvature_file is not specified or empty!");
      return -1;
    }
    if (!velocity_filename_.size())
    {
      ROS_FATAL("Parameter velocity_file is not specified or empty!");
      return -1;
    }

    if (loadWaypoints())
      return -2;
    if (loadCurvature())
      return -2;
    if (loadVelocity())
      return -2;

    heading_pub_ = node_->advertise<kut_ugv_reactive_planner::HeadingStamped>(std::string("heading"), 100);
    motion_pub_ = node_->advertise<kut_ugv_msgs::MotionCommandStamped>(std::string("motion_command"), 100);
    trajectory_pub_ = node_->advertise<nav_msgs::Path>(std::string("trajectory"), 100);

    planner_velocity_pub_ = node_->advertise<kut_ugv_msgs::VelocityLimit>(std::string("planner_velocity"), 100);

    roof_cloud_sub_ = node_->subscribe<sensor_msgs::PointCloud2>("roof_cloud", 100, &ReactivePlannerNode::RoofLaserCloudCallback, this);
    bumper_cloud_sub_ = node_->subscribe<sensor_msgs::PointCloud2>("bumper_cloud", 100, &ReactivePlannerNode::BumperLaserCloudCallback, this);

    laser_sub_ = node_->subscribe<sensor_msgs::LaserScan>("scan", 100, &ReactivePlannerNode::LaserCallback, this);
    //gps_sub_ = node_->subscribe<nav_msgs::Odometry>("gps_xy", 100, &ReactivePlannerNode::GPSCallback, this);
    gps_sub_ = node_->subscribe<nav_msgs::Odometry>("/ekf_odom", 100, &ReactivePlannerNode::GPSCallback, this);
    vehicle_sub_ = node_->subscribe<kut_ugv_vehicle::StateStamped>("vehicle_state", 100, &ReactivePlannerNode::VehicleStateCallback, this);
    velocity_limit_sub_ = node_->subscribe<kut_ugv_msgs::VelocityLimit>("velocity_limit", 100, &ReactivePlannerNode::VelocityLimitCallback, this);

    object_sub_ = node_->subscribe<kut_ugv_sensor_fusion::lidar_object_list>("objects", 100, &ReactivePlannerNode::LidarObjectCallback, this);

    return 0;
  }

  int loadWaypoints()
  {
    std::fstream fs;

    fs.open(wp_filename_.c_str(), std::fstream::in);

    if (!fs.is_open())
    {
      ROS_FATAL("Cannot open waypoints file: %s", wp_filename_.c_str());
      return -1;
    }

    double x_offset = 0.0;
    double y_offset = 0.0;

    wp_.clear();

    do
    {
      double x, y;
      std::string str;
      tf::Point pt;
      std::istringstream ss;

      std::getline(fs, str);
      ss.str(str);
      ss >> x >> y;
      pt.setValue(x+x_offset, y+y_offset, 0.0);
      wp_.push_back(pt);

      //      std::cout << str << "\n";
      //      ROS_INFO("x: %7.7f, y: %7.7f", x, y);
    } while (!fs.eof());

    if (!wp_.size())
      return -2;

    // Everything OK
    return 0;
  }

  int loadCurvature(){
    std::fstream fs;

      fs.open(curvature_filename_.c_str(), std::fstream::in);

      if (!fs.is_open())
      {
        ROS_FATAL("Cannot open curvature file: %s", curvature_filename_.c_str());
        return -1;
      }

      curvature.clear();

      do
      {
        double c;
        std::string str;
        tf::Point pt;
        std::istringstream ss;

        std::getline(fs, str);
        ss.str(str);
        ss >> c;
        pt.setValue(c, 0.0, 0.0);
        curvature.push_back(pt);

        //      std::cout << str << "\n";
        //      ROS_INFO("x: %7.7f, y: %7.7f", x, y);
      } while (!fs.eof());

      if (!curvature.size())
        return -2;

      // Everything OK
    return 0;
  }

  int loadVelocity(){
    std::fstream fs;

      fs.open(velocity_filename_.c_str(), std::fstream::in);

      if (!fs.is_open())
      {
        ROS_FATAL("Cannot open velocity file: %s", curvature_filename_.c_str());
        return -1;
      }

      ref_velocity_.clear();

      do
      {
        double v;
        std::string str;
        tf::Point pt;
        std::istringstream ss;

        std::getline(fs, str);
        ss.str(str);
        ss >> v;
        pt.setValue(v, 0.0, 0.0);
        ref_velocity_.push_back(pt);

        //      std::cout << str << "\n";
        //      ROS_INFO("x: %7.7f, y: %7.7f", x, y);
      } while (!fs.eof());

      if (!ref_velocity_.size())
        return -2;

      // Everything OK
    return 0;
  }

  // Publish data
  void publish()
  {
    ros::Rate loop_rate(publish_rate_);

    while (node_->ok())
    {
      //kut_ugv_msgs::MotionCommandStamped mc;

      kut_ugv_msgs::VelocityLimit velocity_info;

      velocity_info.x = planner_speed_limit_x;
      velocity_info.y = planner_speed_limit_y;
      velocity_info.max_velocity = planner_speed_limit;

      planner_velocity_pub_.publish(velocity_info);

      nav_msgs::Path path;

      path.header.stamp = ros::Time::now();
      path.header.frame_id = "odom";

      int path_length = planning_sample;
      path.poses.resize(path_length);
      path_.resize(path_length);
      //ROS_INFO("path_x: %f, path_y: %f",path_[0].x(),path_[0].y());

      for(int i=0; i<path_length;i++){
        path.poses[i].pose.position.x = path_[i].x();
        path.poses[i].pose.position.y = path_[i].y();
        //path.poses[i].pose.position.x = 0;
        //path.poses[i].pose.position.y = 0;
        path.poses[i].pose.position.z = 0;
      }
      trajectory_pub_.publish(path);

      cur_steering_.header.stamp = ros::Time::now();
      cur_steering_.header.frame_id = "base_link";

      //mc.header = cur_steering_.header;
      //mc.heading_delta = -cur_steering_.heading.data;
      //mc.heading_delta = -angle_diff_;
      //mc.curvature = curvature_;
      //mc.curvature = 0;
      //mc.lateral_offset = lateral_offset_;
      //mc.velocity_limit = speed_command_ * 3600/1000 -2;

      //motion_pub_.publish(mc);
      //heading_pub_.publish(cur_steering_);

      ros::spinOnce();
      loop_rate.sleep();
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reactive_planner");

  ReactivePlannerNode rp_ros;
  if (rp_ros.init())
  {
    ROS_FATAL("ReactivePlannerNode initialization failed");
    return -1;
  }

  rp_ros.publish();

  return 0;
}
