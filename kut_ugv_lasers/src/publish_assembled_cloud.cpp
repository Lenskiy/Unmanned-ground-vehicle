/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <float.h>
#include <ros/ros.h>

#include <laser_assembler/AssembleScans2.h>
#include <sensor_msgs/PointCloud2.h>

#include <kut_ugv_vehicle/StateStamped.h>

/***
 * This a simple test app that requests a point cloud from the
 * point_cloud_assembler every 4 seconds, and then publishes the
 * resulting data
 */
using namespace laser_assembler;

class PublishCloud
{
  ros::NodeHandlePtr n_;
  ros::NodeHandlePtr pn_;
  ros::Publisher pub_;
  ros::Subscriber sub_vel_;
  ros::ServiceClient client_;
  ros::Timer timer_;
  bool first_time_;
  double publish_rate_;
  double distance_horizon_;
  double velocity_;
  ros::Duration time_horizon_;

  // 5.6 m/s
  double default_velocity_;
public:

  PublishCloud():
    first_time_(true), publish_rate_(10.0), distance_horizon_(10.0),
    velocity_(0.0), default_velocity_(5.6)
  {
    n_ = ros::NodeHandlePtr(new ros::NodeHandle);
    pn_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

    // Create a publisher for the clouds that we assemble
    pub_ = n_->advertise<sensor_msgs::PointCloud2> ("assembled_cloud2", 1);

    // Subscribe to velocity topic to determine time horizon
    sub_vel_ = n_->subscribe<kut_ugv_vehicle::StateStamped>("vehicle_state", 100, &PublishCloud::stateCallback, this);

    // Create the service client for calling the assembler
    client_ = n_->serviceClient<AssembleScans2>("assemble_scans2");

    // Publish rate
    pn_->param("publish_rate", publish_rate_, 10.0);

    // Distance horizon in meters at 5.6 m/s (20 km/h). How far we want to see?
    // Will be scaled w.r.t. to current velocity
    pn_->param("distance_horizon", distance_horizon_, 10.0);

    time_horizon_ = ros::Duration(distance_horizon_ / default_velocity_);

    // Start the timer that will trigger the processing loop (timerCallback)
    timer_ = n_->createTimer(ros::Duration(1/publish_rate_), &PublishCloud::timerCallback, this);

    // Need to track if we've called the timerCallback at least once
    first_time_ = true;
  }

  void stateCallback(const kut_ugv_vehicle::StateStampedConstPtr& msg)
  {
    velocity_ = msg->state.velocity;
  }

  void timerCallback(const ros::TimerEvent& e)
  {

    // We don't want to build a cloud the first callback, since we we
    //   don't have a start and end time yet
    if (first_time_)
    {
      first_time_ = false;
      return;
    }

    // Populate our service request based on our timer callback times
    AssembleScans2 srv;

    // Check if velocity is zero
    if((velocity_ - DBL_EPSILON) < 0.0)
      time_horizon_ = ros::Duration(distance_horizon_ / default_velocity_);
    else
      time_horizon_ = ros::Duration(distance_horizon_ / velocity_);

    srv.request.begin = e.current_real - time_horizon_;
    srv.request.end   = e.current_real;

    // Make the service call
    if (client_.call(srv))
    {
      ROS_INFO("Published Cloud with %u points", (uint32_t)(srv.response.cloud.data.size()/srv.response.cloud.point_step)) ;
      pub_.publish(srv.response.cloud);
    }
    else
    {
      ROS_ERROR("Error making service call\n") ;
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "periodic_snapshotter");
  ros::NodeHandle n;
  ROS_INFO("Waiting for [build_cloud2] to be advertised");
  ros::service::waitForService("build_cloud2");
  ROS_INFO("Found build_cloud2! Starting the snapshotter");
  PublishCloud snapshotter;
  ros::spin();
  return 0;
}
