/*
 * behavior_planner.cpp
 *
 *  Created on: Sep 21, 2013
 *      Author: 0xff
 */

#include <algorithm>
#include <vector>

#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <kut_ugv_vehicle/StateStamped.h>
#include <kut_ugv_msgs/WorldObject.h>
#include <kut_ugv_msgs/VelocityLimit.h>

#include "StateMachine.h"

class BehaviorPlanner
{
public:
  int32_t publish_rate_;
  double distance_threshold_;

  ros::NodeHandlePtr node_;
  // Private node for parameters
  ros::NodeHandlePtr pnode_;

  ros::Publisher vlimit_pub_;
  // subscribe to control message
  ros::Subscriber object_sub_;
  ros::Subscriber vehicle_sub_;
  ros::Subscriber odom_sub_;

  std::string tf_prefix_;

  boost::shared_ptr<tf::TransformListener> tf_ls_;

  // Odometry fixed frame
  std::string fixed_frame_;

  typedef std::vector<kut_ugv_msgs::WorldObjectPtr> WorldObjectList;
  typedef WorldObjectList::iterator WorldObjectListIter;
  WorldObjectList objects_list_;

  boost::shared_ptr<StateMachine> sm_;

  BehaviorPlanner() :
      publish_rate_(30), distance_threshold_(3.0)
  {
  }

  int init()
  {
    // Use global namespace for node
    node_ = ros::NodeHandlePtr(new ros::NodeHandle());
    // Get tf_prefix from global namespace
    node_->param("tf_prefix", tf_prefix_, std::string(""));

    // Use private namespace for parameters
    pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));
    pnode_->param("publish_rate", publish_rate_, 30);
    pnode_->param("distance_threshold", distance_threshold_, 3.0);

    pnode_->param("fixed_frame", fixed_frame_, std::string("odom"));
    fixed_frame_ = tf::resolve(tf_prefix_, fixed_frame_);

    // instantiate state machine before any callbacks setup
    sm_ = boost::shared_ptr<StateMachine>(new StateMachine(node_, pnode_));

    vlimit_pub_ = node_->advertise<kut_ugv_msgs::VelocityLimit>("velocity_limit", 10);
    object_sub_ = node_->subscribe<kut_ugv_msgs::WorldObject>("world_object", 10, &BehaviorPlanner::worldObjectCallback,
                                                              this);
    vehicle_sub_ = node_->subscribe<kut_ugv_vehicle::StateStamped>("state", 100, &BehaviorPlanner::vehicleCallback,
                                                                   this);
    odom_sub_ = node_->subscribe<nav_msgs::Odometry>("odom", 100, &BehaviorPlanner::odomCallback, this);

    tf_ls_ = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener);

    return 0;
  }

  // Functor for STL
  struct IsSameObject
  {
    kut_ugv_msgs::WorldObjectPtr new_obj_;
    tf::Point new_p_;
    double distance_threshold_;
    IsSameObject(const kut_ugv_msgs::WorldObjectPtr& obj, double dist) :
        new_obj_(obj), new_p_(obj->pose.position.x, obj->pose.position.y, obj->pose.position.z),
      distance_threshold_(dist)
    {
    }
    bool operator()(const kut_ugv_msgs::WorldObjectPtr& obj)
    {
      // First check if types are same
      if (obj->type == new_obj_->type)
      {
        // if IDs are same, but not 0 then it is the same object
        if (new_obj_->id == obj->id)
        {
          if (new_obj_->id != 0)
          {
            return true;
          }
          // both IDs are 0
          else
          {
            // check the distance between objects
            tf::Point p1(obj->pose.position.x, obj->pose.position.y, obj->pose.position.z);
            if (p1.distance(new_p_) < distance_threshold_)
              return true;
          }
        }
        // IDs are different
        else
        {
          return false;
        }
      }
      else
      {
        // Handle possible misdetection of Max/Min Speed Limit sign
        if(((obj->type == WorldObject::SIGN_SPEED_MAXIMUM) || (obj->type == WorldObject::SIGN_SPEED_MAXIMUM)) &&
           ((new_obj_->type == WorldObject::SIGN_SPEED_MAXIMUM) || (new_obj_->type == WorldObject::SIGN_SPEED_MAXIMUM)))
        {
          // check the distance between objects
          tf::Point p1(obj->pose.position.x, obj->pose.position.y, obj->pose.position.z);
          if (p1.distance(new_p_) < distance_threshold_)
            return true;
        }
      }
      // types are different
      return false;
    }
  };

  void worldObjectCallback(const kut_ugv_msgs::WorldObjectConstPtr& obj)
  {
    geometry_msgs::PoseStamped pose_in;
    geometry_msgs::PoseStamped pose_out;
    kut_ugv_msgs::WorldObjectPtr global_obj = boost::shared_ptr<kut_ugv_msgs::WorldObject>(
        new kut_ugv_msgs::WorldObject(*obj));
    WorldObjectListIter it;

    pose_in.header = obj->header;
    // query latest transform
    pose_in.header.stamp = ros::Time(0);
    pose_in.pose = obj->pose;

    try
    {
      tf_ls_->transformPose(fixed_frame_, pose_in, pose_out);
      global_obj->header = pose_out.header;
      global_obj->pose = pose_out.pose;

      // Check whether it is the same object or not
      it = std::find_if(objects_list_.begin(), objects_list_.end(), IsSameObject(global_obj, distance_threshold_));
      // if is the same, just replace it
      if (it != objects_list_.end())
      {
        *it = global_obj;
      }
      // if not add it to the list
      else
      {
        objects_list_.push_back(global_obj);
        ROS_INFO_STREAM("New object: " << global_obj->type);

        // TODO Sort objects by distance and send the closest one to State Machine

        // TODO Notify state machine about new object
        sm_->update(global_obj, Events::NewObject);
      }
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
    }

  }

  // Functor for STL
  struct IsBehind
  {
    boost::shared_ptr<tf::Transform> t_;
    tf::Point p_;
    boost::shared_ptr<StateMachine> sm_;

    IsBehind(boost::shared_ptr<tf::Transform>& t, boost::shared_ptr<StateMachine>& sm) :
        t_(t), sm_(sm)
    {
    }
    bool operator()(const kut_ugv_msgs::WorldObjectPtr& obj)
    {
      bool result = false;
      tf::Point p_obj(obj->pose.position.x, obj->pose.position.y, obj->pose.position.z);
//      // preserve SPEED sign as it should have a pair
//      if(obj->type == kut_ugv_msgs::WorldObject::SIGN_SPEED_MAXIMUM) return false;
      p_ = t_->inverse() * p_obj;
      // if the point behind, remove it
      result = (p_.x() < 0.0);
      if (result)
      {
        // Notify state machine that we just passed the object
        sm_->update(obj, Events::PassedObject);
        return result;
      }

      return result;
    }
  };

  void odomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    tf::Point p0(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    tf::Point p1;
    tf::Point p1_local;
    boost::shared_ptr<tf::Transform> t(new tf::Transform);

    tf::poseMsgToTF(msg->pose.pose, *t);
    // Update the list of objects. Remove those are behind the vehicle
    objects_list_.erase(std::remove_if(objects_list_.begin(), objects_list_.end(), IsBehind(t, sm_)), objects_list_.end());

    sm_->updateOdometry(msg);
//    if(objects_list_.size())
//      ROS_INFO_STREAM("Objects: " << objects_list_.size());
  }

  void vehicleCallback(const kut_ugv_vehicle::StateStampedConstPtr& state)
  {
    static bool flag = true;
    // ignore backward motion
    if(state->state.velocity < 0.0)
    {
      flag = true;
      return;
    }

    // vehicle is not moving
    if((state->state.velocity - DBL_EPSILON) <= 0.0)
    {
      if(flag)
      {
        sm_->update(WorldObjectPtr(), Events::VehicleStopped);
        flag = false;
      }
    }
    // we move again, reset the flag
    else
    {
      flag = true;
    }
  }

  void run()
  {
    this->init();

    ros::spin();
//    while(node_->ok())
//    {
//
//    }

  }
};

int main(int argc, char** argv)
{
  //-- Init ROS node
  ros::init(argc, argv, "behavior_planner");
  BehaviorPlanner bp_ros;

  bp_ros.run();

  return 0;
}

