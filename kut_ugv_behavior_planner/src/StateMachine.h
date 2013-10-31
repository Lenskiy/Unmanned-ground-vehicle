/*
 * StateMachine.h
 *
 *  Created on: Sep 21, 2013
 *      Author: 0xff
 */

#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

#include <string>

#include <ros/ros.h>
#include <ros/publisher.h>
#include <kut_ugv_msgs/WorldObject.h>

using namespace kut_ugv_msgs;

namespace Missions {
  typedef enum {
    Unknown,
    Start,
    TrafficLightStop,
    TrafficLightDirection,
    DroppedObstacle,
    SpeedLimit,
    BrokenVehicle,
    RoadWorks,
    RandomObstacles,
    Pedestrian,
    NarrowPass,
    CrossRoad,
    Finish,
    _last
  } Mission;
  const std::string MissionNames[_last] = {
    "Unknown",
    "Start",
    "TrafficLightStop",
    "TrafficLightDirection",
    "DroppedObstacle",
    "SpeedLimit",
    "BrokenVehicle",
    "RoadWorks",
    "RandomObstacles",
    "Pedestrian",
    "NarrowPass",
    "CrossRoad",
    "Finish",
  };
}

namespace States {
  typedef enum {
    WaitObject,
    NewObject,
    PassedObject,
    WaitStopLine,
    WaitVehicleToPass,
    WaitPedestrianToPass,
    WaitGreenLight,
    WaitSpeedLimitCancelation,
    WaitTimeOut,
    TrafficLightGreenRed,
    SpeedLimit,
    RoadWorks,
    PedestrianZone,
    NarrowPass,
    CrossRoads,
    Finish,
    _last
  } State;
  const std::string StateNames[_last] = {
    "WaitObject",
    "NewObject",
    "PassedObject",
    "WaitStopLine",
    "WaitVehicleToPass",
    "WaitPedestrianToPass",
    "WaitGreenLight",
    "WaitSpeedLimitCancelation",
    "WaitTimeOut",
    "TrafficLightGreenRed",
    "SpeedLimit",
    "RoadWorks",
    "PedestrianZone",
    "NarrowPass",
    "CrossRoads",
    "Finish",
  };
} // namespace States

namespace Actions {
  typedef enum {
    StopAtPoint,
    SteerLeft,
    SteerRight,
    LimitVelocity,
    Move,
    _last
  } Action;
  const std::string ActionNames[_last] = {
    "StopAtPoint",
    "SteerLeft",
    "SteerRight",
    "LimitVelocity",
    "Move",
  };
}

namespace Events {
  typedef enum {
    None = 0,
    NewObject,
    SameObject,
    PassedObject,
    DistanceTraveled,
    TimeOut,
    VehicleStopped,
    ObjectTraveled,
    _last
  } Event;
  const std::string EventNames[_last] = {
    "None",
    "NewObject",
    "SameObject",
    "PassedObject",
    "DistanceTraveled",
    "TimeOut",
    "VehicleStopped",
    "ObjectTraveled",
  };
} // namespace Events

class ActionContainer {
  double low_velocity_;
public:
  Actions::Action action;
  VelocityLimit velocity;

  ActionContainer(): low_velocity_(5.6), action(Actions::Move){}
  ActionContainer(double vel): low_velocity_(vel), action(Actions::Move){}
  void setAction(const Actions::Action& a, const WorldObjectConstPtr& obj = WorldObjectPtr())
  {
    action = a;
    velocity.type = VelocityLimit::NONE;
    switch(a)
    {
      // TODO !ACHTUNG! Take into account the length of the vehicle!
      case Actions::Move:
        velocity.max_velocity = INFINITY;
        velocity.x = NAN;
        velocity.y = NAN;
        break;
      case Actions::StopAtPoint:
        ROS_ASSERT(obj);
        velocity.max_velocity = 0.0;
        velocity.x = obj->pose.position.x;
        velocity.y = obj->pose.position.y;
        break;
      case Actions::SteerLeft:
        ROS_ASSERT(obj);
        velocity.type = VelocityLimit::STEER_LEFT;
        velocity.max_velocity = INFINITY;
        velocity.x = obj->pose.position.x;
        velocity.y = obj->pose.position.y;
        break;
      case Actions::SteerRight:
        ROS_ASSERT(obj);
        velocity.type = VelocityLimit::STEER_RIGHT;
        velocity.max_velocity = INFINITY;
        velocity.x = obj->pose.position.x;
        velocity.y = obj->pose.position.y;
        break;
      case Actions::LimitVelocity:
        ROS_ASSERT(obj);
        velocity.max_velocity = low_velocity_;
        velocity.x = obj->pose.position.x;
        velocity.y = obj->pose.position.y;
        break;
      default:
        ROS_ASSERT_MSG(false, "Invalid action type!");
    }
  }
};

class StateMachine
{
  double bumper_to_odom_dist_;
  double distance_traveled_;
  double distance_limit_;
  double track_object_traveled_;
  double track_object_limit_;
  ros::Duration time_passed_;

  ros::Duration traffic_light_timeout_;
  ros::Duration pedestrian_timeout_;
  ros::Duration vehicle_timeout_;

  double speed_limit_distance_min_;
  double speed_limit_distance_max_;
  double low_velocity_;
  double stopline_to_traffic_light_;
  double stop_to_pedestrian_;
  double pedestrian_travel_distance_;
  double vehicle_travel_distance_;

  WorldObjectConstPtr stopline_obj_;
  WorldObjectConstPtr light_obj_;
  WorldObjectConstPtr speedbump_obj_;
  WorldObjectConstPtr speed_limit_obj_;
//  WorldObjectPtr pedestrian_obj_;
//  WorldObjectPtr vehicle_obj_;
  tf::Point start_point_;
  WorldObjectConstPtr track_obj_;

  bool vehicle_stopped_;

  nav_msgs::OdometryConstPtr last_odom_;

  Missions::Mission mission_;
  States::State state_;
  ActionContainer ac_;

  ros::Timer timer_;
  ros::Timer distance_watchdog_timer_;
  ros::Timer object_tracking_timer_;

  ros::Publisher ac_pub_;
public:

  StateMachine(const ros::NodeHandlePtr& nh, const ros::NodeHandlePtr& pnh):
      bumper_to_odom_dist_(3.54),
      distance_traveled_(0.0),
      distance_limit_(0.0),
      track_object_traveled_(0.0),
      track_object_limit_(0.0),
      time_passed_(0.0),
      traffic_light_timeout_(40.0),
      pedestrian_timeout_(40.0),
      vehicle_timeout_(40.0),
      speed_limit_distance_min_(20.0), // 20m
      speed_limit_distance_max_(300.0), // 300m
      low_velocity_(20.0 * 1000.0 / 3600.0), // 5.6m/s (20km/h)
      stopline_to_traffic_light_(15.0), // 15m
      stop_to_pedestrian_(5.0), // 5m
      pedestrian_travel_distance_(10.0), // 10m
      vehicle_travel_distance_(60.0), // 60m
      vehicle_stopped_(false),
      mission_(Missions::Unknown), state_(States::WaitObject)
  {
    // Setup timer, oneshot = true, autostart = false
    timer_ = nh->createTimer(ros::Duration(1.0), &StateMachine::timerCb, this, true, false);

    // Periodically check the traveled distance
    // oneshot = false, autostart = false
    distance_watchdog_timer_ = nh->createTimer(ros::Rate(40.0), &StateMachine::distanceWatchdogCb, this, false, false);

    // Periodically check the object
    // oneshot = false, autostart = false
    object_tracking_timer_ = nh->createTimer(ros::Rate(40.0), &StateMachine::objectTrackingCb, this, false, false);

    pnh->param("bumper_to_odom_dist", bumper_to_odom_dist_, 3.54);
    pnh->param("low_velocity", low_velocity_, 5.6);
    double d_tmp = 0.0;
    pnh->param("traffic_light_timeout", d_tmp, 40.0);
    traffic_light_timeout_ = ros::Duration(d_tmp);
    pnh->param("pedestrian_timeout", d_tmp, 40.0);
    pedestrian_timeout_ = ros::Duration(d_tmp);
    pnh->param("vehicle_timeout", d_tmp, 40.0);
    vehicle_timeout_ = ros::Duration(d_tmp);

    pnh->param("speed_limit_distance_min", speed_limit_distance_min_, 10.0);
    pnh->param("speed_limit_distance_max", speed_limit_distance_max_, 10.0);
    pnh->param("stopline_to_traffic_light", stopline_to_traffic_light_, 15.0);
    pnh->param("stop_to_pedestrian", stop_to_pedestrian_, 5.0);
    pnh->param("pedestrian_travel_distance", pedestrian_travel_distance_, 5.0);
    pnh->param("vehicle_travel_distance", vehicle_travel_distance_, 60.0);


    ac_ = ActionContainer(low_velocity_);

    ac_pub_ = nh->advertise<VelocityLimit>("/action", 10);
  }

  void reportMission(Missions::Mission m)
  {
    ROS_INFO_STREAM("Mission [" << Missions::MissionNames[m] << "] over.");
  }

  void resetDistance()
  {
    distance_traveled_ = 0.0;
  }

  void timerSet(const ros::Duration& period)
  {
    timer_.setPeriod(period);
    timer_.start();

    ROS_INFO_STREAM("[" << Missions::MissionNames[mission_] << "]: "
                        << "Timeout set for " << period.toSec() << " seconds");
  }

  void timerCb(const ros::TimerEvent& te)
  {
    this->update(WorldObjectConstPtr(), Events::TimeOut);
    ROS_WARN_STREAM("Mission [" << Missions::MissionNames[mission_] << "] timed out!");
  }

  void distanceWatchdogSet(double dist_limit = 0.0)
  {
    if((dist_limit - DBL_EPSILON) <= 0.0)
    {
      distance_watchdog_timer_.stop();
      return;
    }
    distance_limit_ = dist_limit;
    distance_traveled_ = 0.0;
    distance_watchdog_timer_.start();

    ROS_INFO_STREAM("[" << Missions::MissionNames[mission_] << "]: "
                        << "Distance watchdog set for " << dist_limit << " meters");
  }

  void distanceWatchdogCb(const ros::TimerEvent& te)
  {
    if(distance_traveled_ > distance_limit_)
    {
      distance_watchdog_timer_.stop();
      this->update(WorldObjectConstPtr(), Events::DistanceTraveled);
    }
  }

  void objectTrackingSet(double dist_limit = 0.0)
  {
    if((dist_limit - DBL_EPSILON) <= 0.0 || !track_obj_)
    {
      object_tracking_timer_.stop();
    }
    start_point_ = tf::Point(track_obj_->pose.position.x,
                             track_obj_->pose.position.y,
                             0.0);
    track_object_limit_ = dist_limit;
    track_object_traveled_ = 0.0;
    object_tracking_timer_.start();

    ROS_INFO_STREAM("[" << Missions::MissionNames[mission_] << "]: "
                        << "Object travel watchdog set for " << dist_limit << " meters");
  }

  void objectTrackingCb(const ros::TimerEvent& te)
  {
    tf::Point pt = tf::Point(track_obj_->pose.position.x,
                             track_obj_->pose.position.y,
                             0.0);

    track_object_traveled_ = pt.distance(start_point_);

    if(track_object_traveled_ > track_object_limit_)
    {
      object_tracking_timer_.stop();
      this->update(WorldObjectConstPtr(), Events::ObjectTraveled);
    }
  }

  void updateOdometry(const nav_msgs::OdometryConstPtr& odom)
  {
    // First time here?
    if(!last_odom_) last_odom_ = odom;

    tf::Point p0(last_odom_->pose.pose.position.x, last_odom_->pose.pose.position.y, last_odom_->pose.pose.position.z);
    tf::Point p1(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);

    distance_traveled_ += p1.distance(p0);

    time_passed_ += (odom->header.stamp - last_odom_->header.stamp);

    last_odom_ = odom;
  }

  tf::Point objectInLocalCoordinates(const WorldObjectConstPtr& obj)
  {
    tf::Transform t;
    tf::Point pt;

    tf::poseMsgToTF(last_odom_->pose.pose, t);

    pt = t.inverse() * tf::Point(obj->pose.position.x,
                                 obj->pose.position.y,
                                 0.0);
                                 // reject Z axis
                                 //obj->pose.position.z);

    // TODO HACK: displace all objects as if the odometry frame is on front bumper
    pt.setX(pt.x() - bumper_to_odom_dist_);

    return pt;
  }

  tf::Point pointInGlobalCoordinates(const tf::Point &pt_obj)
  {
    tf::Transform t;
    tf::Point pt;

    tf::poseMsgToTF(last_odom_->pose.pose, t);

    pt = t * pt_obj;

    return pt;
  }

  const WorldObjectConstPtr chooseClosestStopObject(const WorldObjectConstPtr& stopline,
                                                     const WorldObjectConstPtr& traffic_light)
  {
    WorldObjectPtr modified_obj;
    tf::Point pt_stopline;
    tf::Point pt_light;

    if(stopline)
    {
      if(!traffic_light) return stopline;
      pt_stopline = objectInLocalCoordinates(stopline);
    }

    if(traffic_light)
    {
      modified_obj = WorldObjectPtr(new WorldObject(*traffic_light));
      pt_light = objectInLocalCoordinates(traffic_light) -
                           tf::Point(stopline_to_traffic_light_, 0.0, 0.0);

      tf::Point pt_light_g = pointInGlobalCoordinates(pt_light);
      modified_obj->pose.position.x = pt_light_g.x();
      modified_obj->pose.position.y = pt_light_g.y();
      modified_obj->pose.position.z = pt_light_g.z();

      // if there is no stopline detected
      if(!stopline) return modified_obj;
    }

    // if there are both objects detected
    if(stopline && traffic_light)
    {
      // if stopline is closer than (traffic light - margin),
      // then stop at the stopline
      if(pt_stopline.x() < pt_light.x())
      {
        return stopline;
      }
      // otherwise stop ~15-20m before the traffic light
      else
      {
        return modified_obj;
      }
    }

    // We should never reach this point
    // if we did, then there is a bug
    ROS_ASSERT(modified_obj);

    return modified_obj;
  }

  bool isObjectInFrontOfUs(const WorldObjectConstPtr& obj)
  {
    tf::Point pt = objectInLocalCoordinates(obj);

    if(pt.x() > 0.0 && pt.x() < 15.0)
    {
      return (fabs(pt.y()) < 1.0);
    }
    return false;
  }

  const WorldObjectConstPtr displaceObject(const WorldObjectConstPtr& obj, double displacement)
  {
    WorldObjectPtr modified_obj = WorldObjectPtr(new WorldObject(*obj));
    tf::Point pt_local = objectInLocalCoordinates(obj)
                       - tf::Point(displacement, 0.0, 0.0);

    tf::Point pt_global = pointInGlobalCoordinates(pt_local);
    modified_obj->pose.position.x = pt_global.x();
    modified_obj->pose.position.y = pt_global.y();
    modified_obj->pose.position.z = pt_global.z();

    return modified_obj;
  }

  Missions::Mission determineMission(const WorldObjectConstPtr& obj)
  {
    Missions::Mission m = mission_;

    if(mission_ != Missions::Unknown) return mission_;

    switch (obj->type) {
      // Ignore these for now
      case WorldObject::OBSTACLE_UNKNOWN:
      case WorldObject::OBSTACLE_BARRICADE:
      case WorldObject::OBSTACLE_CONE:
        break;
      // We don't know the mission yet, so ignore it
      // It is not safe to assume that object belongs
      // to a mission, if we don't know the mission itself.
      case WorldObject::OBSTACLE_VEHICLE:
      case WorldObject::OBSTACLE_PEDESTRIAN:
        break;
      case WorldObject::OBSTACLE_SPEEDBUMP:
        break;
      case WorldObject::LIGHT_ARROW_LEFT:
      case WorldObject::LIGHT_ARROW_RIGHT:
        m = Missions::TrafficLightDirection;
        break;
      case WorldObject::LIGHT_GREEN:
      case WorldObject::LIGHT_RED:
        m = Missions::TrafficLightStop;
        break;
      // We don't know the mission yet, so ignore it
      case WorldObject::MARK_STOPLINE:
        break;
      case WorldObject::SIGN_CROSSROAD:
        m = Missions::CrossRoad;
        break;
      case WorldObject::SIGN_PEDESTRIAN_ZONE:
        m = Missions::Pedestrian;
        break;
      case WorldObject::SIGN_ROAD_WORKS:
        m = Missions::RoadWorks;
        break;
      // TODO There is a probability of misdetection,
      // so any of the two may open the mission
      case WorldObject::SIGN_SPEED_MAXIMUM:
      case WorldObject::SIGN_SPEED_MINIMUM:
        m = Missions::SpeedLimit;
        break;
    }
    return m;
  }

  Missions::Mission update(const WorldObjectConstPtr& obj = WorldObjectPtr(), Events::Event event = Events::None)
  {
    static uint32_t counter = 0;
    States::State old_state = state_;
    Missions::Mission prev_mission = mission_;

    if(!last_odom_) {
      ROS_WARN_ONCE("Odometry topic is not ready. This message will be printed only once.");
      return mission_;
    }

    if(mission_ == Missions::Unknown)
    {
      if(obj) mission_ = this->determineMission(obj);
    }

    switch(mission_)
    {
      case Missions::Unknown:
        // No objects and no mission? Go!
        ac_.setAction(Actions::Move);
        break;
      case Missions::TrafficLightStop:
        if(event == Events::VehicleStopped)
        {
          // start the timer
          this->timerSet(traffic_light_timeout_);
          break;
        }
        if(event == Events::TimeOut)
        {
          // if we timed out, 'cancel' the mission
          mission_ == Missions::Unknown;
          ac_.setAction(Actions::Move);
          break;
        }
        if(obj)
        {
          if(event == Events::NewObject)
          {
            if(obj->type == WorldObject::LIGHT_RED)
            {
              WorldObjectConstPtr stop_obj;
              light_obj_ = obj;
              // if there is a stopline in front of us as well
              stop_obj = chooseClosestStopObject(stopline_obj_, light_obj_);
              ac_.setAction(Actions::StopAtPoint, stop_obj);
            }
            else if(obj->type == WorldObject::LIGHT_GREEN)
            {
              ac_.setAction(Actions::Move, obj);
            }
            else if(obj->type == WorldObject::MARK_STOPLINE)
            {
              WorldObjectConstPtr stop_obj;
              stopline_obj_ = obj;
              // if there is a traffic light in front of us as well
              stop_obj = chooseClosestStopObject(stopline_obj_, light_obj_);

              ac_.setAction(Actions::StopAtPoint, stop_obj);
            }
          }
          // Events::PassedObject
          else
          {
            switch(obj->type)
            {
              case WorldObject::MARK_STOPLINE:
                // reset pointer to NULL
                stopline_obj_ = WorldObjectPtr();
                break;
              case WorldObject::LIGHT_RED:
              case WorldObject::LIGHT_GREEN:
                // reset pointer to NULL
                light_obj_ = WorldObjectPtr();
                // Mission is over
                mission_ = Missions::Unknown;
                ac_.setAction(Actions::Move);
                break;
            }
          }
        } // end if(obj)
        break;
      case Missions::TrafficLightDirection:
        if(obj)
        {
          if(event == Events::NewObject)
          {
            if(obj->type == WorldObject::LIGHT_ARROW_LEFT)
            {
              ac_.setAction(Actions::SteerLeft, obj);
            }
            else if(obj->type == WorldObject::LIGHT_ARROW_RIGHT)
            {
              ac_.setAction(Actions::SteerRight, obj);
            }
          }
          else
          {
            mission_ = Missions::Unknown;
            ac_.setAction(Actions::Move);
          }
        }
        break;
      case Missions::SpeedLimit:
        switch(event)
        {
          case Events::NewObject:
            if(!speed_limit_obj_){
              speed_limit_obj_ = obj;
              ac_.setAction(Actions::LimitVelocity, obj);
            }
            else speed_limit_obj_ = WorldObjectPtr();
            break;
          case Events::SameObject:
            // If this is the first sign, update pointer,
            // otherwise ignore
            if(speed_limit_obj_)
            {
              speed_limit_obj_ = obj;
              // Update action position
              ac_.setAction(Actions::LimitVelocity, obj);
            }
            break;
          case Events::PassedObject:
            // We just passed by the first speed limit traffic sign
            if(speed_limit_obj_)
            {
              // Accumulate traveled distance from here
              this->distanceWatchdogSet(speed_limit_distance_max_);
            }
            // if it is the second sign, then finalize the mission
            else
            {
              // stop watchdog
              this->distanceWatchdogSet(0.0);
              mission_ = Missions::Unknown;
              ac_.setAction(Actions::Move);
            }
            break;
          case Events::DistanceTraveled:
            // 'cancel' the mission
            mission_ = Missions::Unknown;
            ac_.setAction(Actions::Move);
            break;
        }
        break;
      case Missions::BrokenVehicle:
        break;
      case Missions::RoadWorks:
        // Cancel whatever mission it was after we pass the sign
        // TODO can be unsafe, need to start distance watchdog
        if(event == Events::PassedObject){
          mission_ = Missions::Unknown;
          ac_.setAction(Actions::Move);
        }
        break;
      case Missions::RandomObstacles:
        break;
      case Missions::Pedestrian:
        // We are in pedestrian zone
        switch(event)
        {
          case Events::NewObject:
            if(obj->type == WorldObject::OBSTACLE_PEDESTRIAN)
            {
              track_obj_ = obj;
              WorldObjectConstPtr stop_obj = displaceObject(obj, stop_to_pedestrian_);
              ac_.setAction(Actions::StopAtPoint, obj);
            }
            break;
          case Events::SameObject:
            if(obj->type == WorldObject::OBSTACLE_PEDESTRIAN)
            {
              track_obj_ = obj;
              WorldObjectConstPtr stop_obj = displaceObject(obj, stop_to_pedestrian_);

              // still moving?
              if(!vehicle_stopped_)
              {
                ac_.setAction(Actions::StopAtPoint, obj);
              }
              else
              {
                objectTrackingSet(pedestrian_travel_distance_);
              }
            }
            break;
          case Events::PassedObject:
            switch(obj->type)
            {
              case WorldObject::SIGN_PEDESTRIAN_ZONE:
                // TODO We may need to slow down here
                break;
              case WorldObject::OBSTACLE_PEDESTRIAN:
                // Finalize the mission
                mission_ == Missions::Unknown;
                ac_.setAction(Actions::Move);
                break;
            }
            break;
          case Events::VehicleStopped:
            vehicle_stopped_ = true;
            // start the timer
            this->timerSet(pedestrian_timeout_);
            break;
          case Events::ObjectTraveled:
            // Make sure there is no obstacle (aka pedestrian) in front of us
            if(isObjectInFrontOfUs(track_obj_) && counter < 1)
            {
              counter++;
              // wait a bit more
              this->timerSet(pedestrian_timeout_ * 0.5);
            } else {
              mission_ == Missions::Unknown;
              ac_.setAction(Actions::Move);
            }
            break;
          case Events::TimeOut:
            ROS_WARN_STREAM("Mission [" << Missions::MissionNames[mission_] << "] timed out!");
            // if we timed out, 'cancel' the mission
            // Make sure there is no obstacle (aka pedestrian) in front of us
            if(isObjectInFrontOfUs(track_obj_) && counter < 1)
            {
              counter++;
              // wait a bit more
              this->timerSet(pedestrian_timeout_ * 0.5);
            } else {
              mission_ == Missions::Unknown;
              ac_.setAction(Actions::Move);
            }
            break;
        }
        break;
      case Missions::NarrowPass:
        break;
      case Missions::CrossRoad:
        break;
      case Missions::Finish:
        break;
      default:
        ROS_ASSERT(mission_ >= 0 && mission_ < Missions::_last);
        ROS_ASSERT_MSG(false, "[%s] is not implemented", Missions::MissionNames[mission_].c_str());
    }

    if(prev_mission != mission_)
    {
      ROS_INFO_STREAM("[" << Missions::MissionNames[prev_mission] << "] -> ["
                          << Missions::MissionNames[mission_] << "]: Event::"
                          << Events::EventNames[event] << " lead to Action::"
                          << Actions::ActionNames[ac_.action]);
    }
    else
    {
      ROS_INFO_STREAM("[" << Missions::MissionNames[mission_] << "]: Event::"
                          << Events::EventNames[event] << " lead to Action::"
                          << Actions::ActionNames[ac_.action]);
    }

    this->publish();

    return mission_;
  }

  void publish()
  {
    ac_pub_.publish(ac_.velocity);
  }
};

#endif /* STATEMACHINE_H_ */
