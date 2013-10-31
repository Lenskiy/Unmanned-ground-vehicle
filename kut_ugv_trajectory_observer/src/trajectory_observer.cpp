#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <kut_ugv_reactive_planner/HeadingStamped.h>
#include <kut_ugv_msgs/MotionCommandStamped.h>
#include <kut_ugv_vehicle/StateStamped.h>
#include <kut_ugv_vehicle/State.h>
#include <kut_ugv_msgs/VelocityLimit.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>


#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>

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
#define num_path 7
#define lat_off_rate 10
#define GPStoLaser 3.66



//#define wp_size 2565 // F1
//#define wp_size 2178 // KUT
#define wp_size 4600 // KATECH

  double behavior_velocity_x_ = 0;
  double behavior_velocity_y_ = 0;
  double behavior_velocity_max_ = 50.0/3.6;
  double action_type_ = 0;


class TrajectoryObserverNode
{
public:



  std::ofstream heading_file_;
  std::ofstream log_file_;
  std::vector<tf::Point> wp_;
  std::vector<tf::Point> path_;

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




  double planner_velocity_x_;
  double planner_velocity_y_;
  double planner_velocity_;

  double lookahead_x;
  double lookahead_y;

  double gps_sec;
  double gps_nsec;

  double sample_time;
  //double old_steering_angle;

  int heading_condition;
  int32_t publish_rate_;
  std::string wp_filename_;
  std::string curvature_filename_;
  std::string velocity_filename_;
  std::string log_filename_;
  std::string log_filename2_;


  ros::NodeHandlePtr node_;
  ros::NodeHandlePtr pnode_;

  ros::Publisher heading_pub_;
  ros::Publisher motion_pub_;
  ros::Publisher map_pub_;
  ros::Subscriber waypoint_sub_;
  ros::Subscriber curvature_sub_;
  ros::Subscriber laser_sub_;
  ros::Subscriber gps_sub_;
  ros::Subscriber vehicle_sub_;
  ros::Subscriber trajectory_sub_;
  ros::Subscriber velocity_limit_sub_;
  ros::Subscriber planner_velocity_sub_;


  //kut_ugv_reactive_planner::HeadingStamped cur_steering_;


  TrajectoryObserverNode() :
    current_heading_(0.0), publish_rate_(100)
  {
    //    this->init();
  }
  ~TrajectoryObserverNode()
  {
    heading_file_.close();
  }

  void TrajectoryCallback(const nav_msgs::PathConstPtr& msg){

    int length = msg->poses.size();
    tf::Point pt;

    path_.clear();

    for(int i=0;i<length;i++){

      double x;
      double y;

      x = msg->poses[i].pose.position.x;
      y = msg->poses[i].pose.position.y;

      pt.setValue(x, y, 0.0);
      path_.push_back(pt);
    }
    //ROS_INFO("%f %f",msg->poses[0].pose.position.x, msg->poses[0].pose.position.y);
  }

  void VelocityLimitCallback(const kut_ugv_msgs::VelocityLimitConstPtr& msg){

    behavior_velocity_x_ = msg ->x;
    behavior_velocity_y_ = msg ->y;
    behavior_velocity_max_ = msg ->max_velocity;
    action_type_ = msg ->type;
    //ROS_INFO("message_get");
  }

  void PlannerVelocityCallback(const kut_ugv_msgs::VelocityLimitConstPtr& msg){

    planner_velocity_x_ = msg ->x;
    planner_velocity_y_ = msg ->y;
    planner_velocity_ = msg ->max_velocity;

    //ROS_INFO("%f",planner_velocity_);

  }


  void VehicleStateCallback(const kut_ugv_vehicle::StateStampedConstPtr& msg){
    crio_speed_ = msg->state.velocity;
    theta_ = msg ->state.theta;
    beta_ = msg->state.beta;

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

    double lookahead = 3.5;




    gps_pt_.setValue(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0);

    tf::Pose pose;
    tf::poseMsgToTF(msg->pose.pose, pose);

    double tmp, Yaw;
    pose.getBasis().getRPY(tmp, tmp, Yaw);






    if (gps_pt_.distance(gps_old_pt) > 0.03)
    {
      if (gps_old_pt != gps_pt_)
      {
        //current_heading_ = tf::Vector3(gps_pt_ - gps_old_pt).angle(tf::Vector3(1.0, 0.0, 0.0));
        current_heading_ = atan2(gps_pt_.y() - gps_old_pt.y(), gps_pt_.x() - gps_old_pt.x());
        heading_condition = 1;
      }
    }

    else
    {
      current_heading_ = 0;
    }
    if (gps_old_pt != gps_pt_){
      speed_ = gps_pt_.distance(gps_old_pt)/100*3600;
    }



    //// Current Heading ////

      heading = Yaw;
      current_heading_ = heading;


      lookahead_x = gps_pt_.x() + lookahead*cos(heading);
      lookahead_y = gps_pt_.y() + lookahead*sin(heading);


      //// Closest Way Point ////

      int current;
      int lookahead_current;
      static int old_current;


      current = ClosetWaypoint_order(old_current, gps_pt_.x(), gps_pt_.y(), current_heading_);
      lookahead_current = ClosetWaypoint_order(old_current, lookahead_x, lookahead_y, current_heading_);


      old_current = current;


      int t_lookahead_point;

      t_lookahead_point = T_closestwaypoint(lookahead_x, lookahead_y, current_heading_);


      ////////------------ Motion ------------////////


      //// Angle Difference ////

      double r_path_angle = 0.0;
      double ref_angle_diff;

      //angle_diff_ = angle_difference(current, current_heading_);
      //angle_diff_ = angle_difference(lookahead_current, current_heading_);
      //ref_angle_diff = angle_difference(lookahead_current, current_heading_);
      angle_diff_ = T_angle_difference(t_lookahead_point, current_heading_);

      if (angle_diff_> M_PI){
        angle_diff_ = angle_diff_ - 2*M_PI;
      }
      else if (angle_diff_< -M_PI){
        angle_diff_ = angle_diff_ + 2*M_PI;
      }
      if (speed_< 1){
        angle_diff_ = 0;
      }


      //// Lateral Offset ////

      double ref_latoffset;
      static double offset = 0.0;

      double ref_lat_offset;


      //lateral_offset_ = lateraloffset(gps_pt_.x(), gps_pt_.y(), wp_[current].x(), wp_[current].y(), wp_[current+1].x(), wp_[current+1].y());
      //ref_latoffset = lateraloffset(lookahead_x, lookahead_y, wp_[lookahead_current].x(), wp_[lookahead_current].y(), wp_[lookahead_current+1].x(), wp_[lookahead_current+1].y());
      //lateral_offset_ = lateraloffset(lookahead_x, lookahead_y, trajectory_[path_choice][t_lookahead_point][1], trajectory_[path_choice][t_lookahead_point][2], trajectory_[path_choice][t_lookahead_point+1][1], trajectory_[path_choice][t_lookahead_point+1][2]);

      if (path_.size() > 1){
        if (t_lookahead_point >= path_.size()){
          int a = path_.size();
          lateral_offset_ = lateraloffset(lookahead_x, lookahead_y, path_[a-1].x(), path_[a-1].y(), path_[a].x(), path_[a].y());
        }
        else{
          lateral_offset_ = lateraloffset(lookahead_x, lookahead_y, path_[t_lookahead_point].x(), path_[t_lookahead_point].y(), path_[t_lookahead_point+1].x(), path_[t_lookahead_point+1].y());
        }
      }


      if (abs(lateral_offset_)>15){
        //lateral_offset_ = 0;
      }

      //// Curvature ////

      static double curvature_c;
      double x_d;
      double y_d;
      static double x_dd;
      static double y_dd;
      static double x_d_old;
      static double y_d_old;


      curvature_=curvature[lookahead_current].x();

      //ROS_INFO("lookahead_point: %d, angle diff: %f, lateral_offset: %f, curvature: %f", t_lookahead_point, angle_diff_, lateral_offset_, curvature_);
      //ROS_INFO("ref angle diff: %f, ref lat_offset: %f, curvature: %f", ref_angle_diff, ref_latoffset, curvature_);

      //log_file_ <<curvature_<<" "<<curvature_c<<"\n";
      //log_file_ <<x_d<<" "<<y_d<<" "<<x_dd<<" "<<y_dd<<" "<<curvature_c<<"\n";

    if (sec == old_sec){
      sample_time = (nsec - old_nsec)/pow(10,9);
    }
    else{
      sample_time = (nsec+pow(10,9) - old_nsec)/pow(10,9);
    }


    static double dt;
    static double current_speed;

    // Speed Control //

    static double ref_speed;
    if (sample_time > 0){
      dt = sample_time;
      //ROS_INFO("current speed: %f",current_speed);
      //speed_command_ =speed_planning(current,current_speed,dt);
      speed_command_ =speed_planning2(current,crio_speed_,dt);
    }


    gps_old_pt = gps_pt_;
    old_sec = sec;
    old_nsec = nsec;

    loop_count+=1;

  }



  int T_closestwaypoint (double gpsx, double gpsy, double heading){

    int order;
    double dist;
    double min_dist = DBL_MAX;

    int length = path_.size();

    if (length > 1){
      for (int i=0; i<length;i++)
      {
        //dist = sqrt(pow((gpsx - trajectory_[path][i][1]),2) + pow((gpsy - trajectory_[path][i][2]),2));
        dist = sqrt(pow((gpsx - path_[i].x()),2) + pow((gpsy - path_[i].y()),2));

        if (min_dist > dist)
        {
          min_dist = dist;
          order = i;
        }
      }
    }
    else{
      order = 0;
    }
    return order;
  }

  int closestwaypoint2 (int old_current, double gpsx, double gpsy, double heading){
    int order;
    double dist;
    double min_dist = DBL_MAX;
    int over_order;

    if (old_current < 2){
      old_current = 2;
    }

    for (int i=old_current-2; i<old_current+100;i++)
    {
      if (i > wp_.size()){
        over_order = i-wp_.size();
        dist = sqrt(pow(gpsx-wp_[over_order].x(),2) + pow(gpsy-wp_[over_order].y(),2));
      }
      else{
      dist = sqrt(pow(gpsx-wp_[i].x(),2) + pow(gpsy-wp_[i].y(),2));
      }
      if (min_dist > dist)
      {
        min_dist = dist;
        order = i;
        if(order > wp_.size()){
          order = i-wp_.size();
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
      //dist = vehicle_front.distance(wp_[j]);

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


  double speed_planning(int current, double current_speed, double dt){

      double speed;
      double goal_speed;
      double goal_dist;
      double min_speed = DBL_MAX;
      double acc;
      double distance;
      double total_dist;
      double ref_speed;

      double lookahead = 40.0;

      dt = 0.05;

      int iter = 0;

      double behavior_x;
      double behavior_y;
      double behavior_v_max = 50.0;
      double behavior_v_min = 0.0;

      double planner_v_max = 40.0;

      double speed_limit;

      behavior_x = behavior_velocity_x_;
      behavior_y = behavior_velocity_y_;

      if (behavior_velocity_max_ <= DBL_EPSILON){
        behavior_v_max = 0.0;
      }
      else{
        behavior_v_max = (behavior_velocity_max_ - 2.0);
      }

      if ( std::isnan(behavior_x) != 0 || std::isnan(behavior_y) != 0){ // if position data is a NaN
        behavior_x = 0;
        behavior_y = 0;
      }


      int object_order;

      int velocity_order;

      if (behavior_x != 0 && behavior_y != 0 ){

        //object_order = closestwaypoint2(current, behavior_x, behavior_y, 0);
        object_order = object_closest(behavior_x, behavior_y);
        object_order =1;
        if (object_order < current){
          //ROS_FATAL("object is on the backward from vehicle!");
          while(total_dist < lookahead){
            distance = wp_[current+iter].distance(wp_[current+iter+1]);
            total_dist += distance;
            velocity_order = current+iter;
            if( velocity_order > ref_velocity_.size()){
              velocity_order = velocity_order - ref_velocity_.size();
            }
            ref_speed = ref_velocity_[velocity_order].x();
            if (ref_speed < min_speed){
              min_speed = ref_speed;
              goal_dist = total_dist;
            }
            iter += 1;
          }
          goal_speed = min_speed;

          speed_limit = planner_v_max;

        }

        else{

          if (abs(object_order-current) < 6){
            speed_limit = behavior_v_max;
          }
          else{
            speed_limit = planner_v_max;
          }

          for(int i=current;i<object_order;i++){
            distance = wp_[i].distance(wp_[i+1]);
            total_dist += distance;
          }
          goal_speed = behavior_v_max;
          goal_dist = total_dist;
        }
        //ROS_INFO("vehicle_position: %d, goal_position: %d, limit: %f",current, object_order, speed_limit);

      }


      else{
        //ROS_INFO("No object information");

        while(total_dist < lookahead){
          distance = wp_[current+iter].distance(wp_[current+iter+1]);
          total_dist += distance;
          ref_speed = ref_velocity_[current+iter].x();
          if (ref_speed < min_speed){
            min_speed = ref_speed;
            goal_dist = total_dist;
          }
          iter += 1;
        }
        goal_speed = min_speed;

        if (behavior_v_max < planner_v_max){
          speed_limit = behavior_v_max;
        }
        else{
          speed_limit = planner_v_max;
        }

      }


      if(planner_velocity_ < 20){
        tf::Point planner_speed_goal;
        planner_speed_goal.setValue(planner_velocity_x_,planner_velocity_y_ ,0);
        goal_speed = planner_velocity_;
        goal_dist = gps_pt_.distance(planner_speed_goal);
      }
      //ROS_INFO("planner_velocity: %f",planner_velocity_);


      goal_speed = goal_speed*1000/3600; //m/s
      acc = (pow(goal_speed,2) - pow(current_speed,2))/(2*(goal_dist));

      double acc_gain = 1.0;
      double acc_limit = 5.0;
      if (acc > 10.0){
        acc = 10.0;
      }
      if (acc > 0 && acc < acc_limit){
        acc = acc_limit;
      }
      if (acc < 0 && acc > -acc_limit){
        acc = -acc_limit;
      }

      if ( acc < 0 ){
        acc = 2.0*acc;
      }


      if (abs(current_speed - goal_speed) > 10/3.6){
        acc = acc*3;
      }

      speed = current_speed + acc*dt;

      if (speed > speed_limit*1000/3600){
        speed = speed_limit*1000/3600;
      }
      else if (speed < 0){
        speed = behavior_v_min*1000/3600;
      }

      // To check the data
      //ROS_INFO("%d current: %f, goal: %f, command: %f, acc: %f, limit: %f", current,current_speed*3600/1000, goal_speed*3600/1000,speed*3600/1000,acc,speed_limit);
      //log_file_.precision(10);
      //log_file_ << ref_velocity_[current].x() <<" "<< speed*3.6 <<" "<<goal_speed*3.6<<" "<< acc << " "<<goal_dist<<"\n";

      return speed;
    }


  double speed_planning2(int current, double current_speed, double dt){

       double speed;

       double acc;

       double ref_speed;



       double behavior_data[2];
       double planner_data[2];
       double reference_data[2];

       int speed_type = 0;

       //--- Get reference goal velocity  and goal position ---//


       double lookahead = 40.0;
       double min_speed = DBL_MAX;

       double ref_min_velocity;
       int reference_veolcity_order;

       int iter = 0;
       double total_ref_dist = 0;
       double distance;


       while(total_ref_dist < lookahead){

         if( (current+iter+1) >= wp_.size()-2){
           min_speed = ref_velocity_[current+iter].x();
           reference_veolcity_order = wp_.size()-2;
           //min_speed = 0;
           break;
         }
         distance = wp_[current+iter].distance(wp_[current+iter+1]);
         total_ref_dist += distance;
         ref_speed = ref_velocity_[current+iter].x();
         if (ref_speed < min_speed){
           min_speed = ref_speed;
           reference_veolcity_order = current+iter;
         }
         iter += 1;
       }
       ref_min_velocity = min_speed;

       reference_data[0] = ref_min_velocity;
       reference_data[1] = reference_veolcity_order;




       //--- Get behavior planner velocity and position ---//


       double behavior_velocity;
       double behavior_x;
       double behavior_y;
       double behavior_v_min = 0.0;


       behavior_x = behavior_velocity_x_;
       behavior_y = behavior_velocity_y_;

       if (behavior_velocity_max_ <= DBL_EPSILON){
         behavior_velocity = 0.0;
       }
       else{
         behavior_velocity = behavior_velocity_max_;
       }
       if ( std::isnan(behavior_x) != 0 || std::isnan(behavior_y) != 0){ // if position data is a NaN
         behavior_x = 0;
         behavior_y = 0;
       }

       behavior_velocity = behavior_velocity*3.6;


       int object_order;
       object_order = object_closest(behavior_x, behavior_y);

       behavior_data[0] = behavior_velocity;
       behavior_data[1] = object_order;



       //--- Get path planner velocity and position ---//

       int planner_velocity_order;
       planner_velocity_order = object_closest(planner_velocity_x_,planner_velocity_y_);

       planner_data[0] = planner_velocity_;
       planner_data[1] = planner_velocity_order;



       //--- Select minimum speed ---//

       double goal_speed;
       double goal_dist;
       double speed_limit = 20.0;
       double speed_min_limit = 3.0;
       int goal_order;



       if ((reference_data[0] <= behavior_data[0]) && (reference_data[0] <= planner_data[0]) ){
         goal_speed = reference_data[0];
         goal_order = reference_data[1];
         speed_type =1;
       }

       else if ((behavior_data[0] <= reference_data[0]) && (behavior_data[0] <= planner_data[0]) ){

         goal_speed = behavior_data[0];
         goal_order = behavior_data[1];

         if (behavior_x == 0 && behavior_y == 0 ){
           speed_limit = behavior_velocity;
         }
         else{
           if (abs(current-goal_order)< 5 ){
             speed_limit = behavior_velocity;
           }
         }
         speed_type = 2;
       }

       else if ((planner_data[0] <= reference_data[0]) && (planner_data[0] <= behavior_data[0]) ){
         goal_speed = planner_data[0];
         goal_order = planner_data[1];
         speed_type = 3;
       }

       else{
         goal_speed = reference_data[0];
         goal_order = reference_data[1];
         speed_type = 1;
       }

       if (goal_order == current ){
          if (current == wp_.size()){
             current = current -1;
          }
          else{
              goal_order = current+1;
          }
       }


       //if( goal_order > current)
       double total_dist = 0;

       if ( (goal_order < current) && goal_speed !=0 ){
         goal_speed = reference_data[0];
         goal_order = reference_data[1];
         speed_type = 1;
       }


       for(int i=current;i<goal_order;i++){
         distance = wp_[i].distance(wp_[i+1]);
         total_dist += distance;
       }
       goal_dist = total_dist;


       //--- Calculate acceleration or deceleration ---//


       goal_speed = goal_speed*1000/3600; //m/s


       acc = (pow(goal_speed,2) - pow(current_speed,2))/(2*(goal_dist));

       double acc_limit = 4.0;

       if (acc > 10.0){
         acc = 10.0;
       }
       if((goal_speed - current_speed > 10/3.6) ){
         acc = acc*2;
       }

       if (acc > 0 && acc < acc_limit){
         acc = acc_limit;
       }
       if (acc < 0 && acc > -acc_limit){
         acc = -acc_limit;
       }

       if ( acc < 0 ){
         acc = 2.0*acc;
       }

       if((goal_speed - current_speed < -20/3.6) && goal_speed == 0){
         acc = acc*14;
       }
       else if( (goal_speed - current_speed < -15/3.6) && goal_speed == 0 && goal_dist < 20 ){
         acc = acc*9;
       }
       else if( (goal_speed - current_speed < -10/3.6) && goal_speed == 0 && goal_dist < 10){
         acc = acc*4;
       }
       else if( (goal_speed - current_speed < -5/3.6) && goal_speed == 0){
         acc = 1*acc;
       }




       double speed_error;

       speed_error = (goal_speed - current_speed)*3.6;

       //if (speed_error < 0){
       //   acc = acc * (-speed_error/1.5);
       //}


       dt = 0.05;



      if (goal_speed == 0 && current > goal_order){
        speed = 0;
      }

      else{
       speed = current_speed + acc*dt;
       }

       /*if(current < goal_order-5){
         speed_min_limit = 3.0;
       }
       else if(goal_speed == 0){
         speed_min_limit = 0.0;
       }*/

       if((current > goal_order-5) && goal_speed == 0 ){
         speed_min_limit = 0.0;
       }
       else{
         speed_min_limit = 3.0;
       }

      if(current > wp_.size()-10){
          //speed = ref_velocity_[1].x()*1000/3600;
          speed = 0;
       }


       if (speed > speed_limit*1000/3600){
         speed = speed_limit*1000/3600;
       }
       else if(speed < speed_min_limit*1000/3600){
         speed = speed_min_limit*1000/3600;
       }
       else if (speed < 0){
         speed = behavior_v_min*1000/3600;
       }




       // To check the data
       ROS_INFO("%d %d %d, cur: %f, goal: %f, cmd: %f, goal_d: %f, a: %f, lmt: %f",speed_type, current,goal_order, current_speed*3600/1000, goal_speed*3600/1000,speed*3600/1000,goal_dist, acc,speed_limit);

       //log_file_.precision(10);
       //log_file_ << ref_velocity_[current].x() <<" "<< speed*3.6 <<" "<<goal_speed*3.6<<" "<< acc << " "<<goal_dist<<"\n";

       return speed;
     }



  double T_angle_difference(int current, double heading){

    double r_path_angle;
    double angle_diff;
    int t_length = path_.size();

    if (path_.size()>1){
      if ( current >= t_length){
        r_path_angle = atan2(path_[t_length].y() - path_[t_length-1].y(), path_[t_length].x() - path_[t_length-1].x());
      }
      else{
        r_path_angle = atan2(path_[current+1].y() - path_[current].y(), path_[current+1].x() - path_[current].x());
      }
      angle_diff = r_path_angle - heading;
    }
    else{
      angle_diff = 0.0;
    }
    return angle_diff;
  }

  double angle_difference(int current, double heading){

    double r_path_angle;
    double angle_diff;
    int length = wp_.size();

    if ( (current+1) > length){
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

    //heading_pub_ = node_->advertise<kut_ugv_reactive_planner::HeadingStamped>(std::string("heading"), 100);
    motion_pub_ = node_->advertise<kut_ugv_msgs::MotionCommandStamped>(std::string("motion_command"), 100);


    //gps_sub_ = node_->subscribe<nav_msgs::Odometry>("gps_xy", 100, &TrajectoryObserverNode::GPSCallback, this);
    gps_sub_ = node_->subscribe<nav_msgs::Odometry>("/ekf_odom", 100, &TrajectoryObserverNode::GPSCallback, this);
    vehicle_sub_ = node_->subscribe<kut_ugv_vehicle::StateStamped>("vehicle_state", 100, &TrajectoryObserverNode::VehicleStateCallback, this);
    trajectory_sub_ = node_->subscribe<nav_msgs::Path>("path_trajectory", 100, &TrajectoryObserverNode::TrajectoryCallback, this);
    planner_velocity_sub_ = node_->subscribe<kut_ugv_msgs::VelocityLimit>("planner_speed_limit", 100, &TrajectoryObserverNode::PlannerVelocityCallback, this);

    velocity_limit_sub_ = node_->subscribe<kut_ugv_msgs::VelocityLimit>("velocity_limit", 100, &TrajectoryObserverNode::VelocityLimitCallback, this);

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
      pt.setValue(x, y, 0.0);
      wp_.push_back(pt);
      // TODO
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
        // TODO
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
        // TODO
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
      kut_ugv_msgs::MotionCommandStamped mc;

      //cur_steering_.header.stamp = ros::Time::now();
      //cur_steering_.header.frame_id = "base_link";

      //mc.header = cur_steering_.header;
      mc.header.stamp = ros::Time::now();
      mc.header.frame_id = "base_link";
      //mc.heading_delta = -cur_steering_.heading.data;
      mc.heading_delta = -angle_diff_;
      //mc.curvature = curvature_;
      mc.curvature = 0;
      mc.lateral_offset = lateral_offset_;
      mc.velocity_limit = speed_command_ * 3600/1000 -2;

      motion_pub_.publish(mc);
      //heading_pub_.publish(cur_steering_);


      ros::spinOnce();
      loop_rate.sleep();
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_observer");

  TrajectoryObserverNode rp_ros;
  if (rp_ros.init())
  {
    ROS_FATAL("TrajectoryObserverNode initialization failed");
    return -1;
  }

  rp_ros.publish();

  return 0;
}
