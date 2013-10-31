#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <gps_common/GPSFix.h>
#include <kut_ugv_vehicle/StateStamped.h>

using namespace nav_msgs;
using namespace kut_ugv_vehicle;
using namespace gps_common;
using namespace message_filters;

void callback(const OdometryConstPtr& gps_msg, const StateStampedConstPtr& state_msg, const GPSFixConstPtr& fix_msg)
{
  static ros::Time t = gps_msg->header.stamp;
  static Odometry last_gps = *gps_msg;
  double x = last_gps.pose.pose.position.x;
  double y = last_gps.pose.pose.position.y;

//  if(x != gps_msg->pose.pose.position.x && y != gps_msg->pose.pose.position.y)
//  {
    printf("%3.8f,%3.8f,%3.8f,%3.8f,%3.8f,%3.8f,%3.8f,%3.8f,%3.8f\n",
	(gps_msg->header.stamp - t).toSec(),
	gps_msg->pose.pose.position.x, gps_msg->pose.pose.position.y,
	gps_msg->pose.covariance[0], gps_msg->pose.covariance[7], state_msg->state.velocity, state_msg->state.theta,
	fix_msg->speed, fix_msg->err_speed);
//  } else {
    
//  }

  t = gps_msg->header.stamp;
  x = gps_msg->pose.pose.position.x;
  y = gps_msg->pose.pose.position.y;

  last_gps = *gps_msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sync_gps_vehicle_state");

  ros::NodeHandle nh;
  message_filters::Subscriber<Odometry> gps_sub(nh, "/gps_xy", 100);
  message_filters::Subscriber<StateStamped> state_sub(nh, "/vehicle/state", 100);
  message_filters::Subscriber<GPSFix> gps_fix_sub(nh, "/gps/extended_fix", 100);

  typedef sync_policies::ApproximateTime<Odometry, StateStamped, GPSFix> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), gps_sub, state_sub, gps_fix_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  printf("t,x,y,x_cov,y_cov,vel,th,vel_gps,vel_gps_cov\n");

  ros::spin();

  return 0;
}
