/*
 * vehicle_kinematics.hpp
 *
 *  Created on: May 31, 2013
 *      Author: 0xff
 */

#ifndef VEHICLE_KINEMATICS_HPP_
#define VEHICLE_KINEMATICS_HPP_

#include <kut_ugv_vehicle/StateStamped.h>

using namespace kut_ugv_vehicle;

class VehicleKinematics
{
public:
  typedef struct Parameters_
  {
    // Frame ID of c.g. (or center of rotation) of the car
    std::string frame_id;
    // Wheelbase length
    tfScalar wheelbase_length;
    // Track width
    tfScalar track_width;
    // Distance from front axle to c.r.
    tfScalar rotation_center;
    // Minimum turning radius (average)
    tfScalar minimum_radius;
    // Steering wheel to wheels rotation ratio
    tfScalar steering_ratio;

    friend std::ostream& operator<<(std::ostream& os, const Parameters_& p)
    {
      os << "Constant parameters: " << "\n" << "\t" << "frame_id: " << p.frame_id << "\n" << "\t" << "wheelbase: "
          << p.wheelbase_length << "\n" << "\t" << "track: " << p.track_width << "\n" << "\t" << "rotation_center: "
          << p.rotation_center << "\n" << "\t" << "minimum_radius_avg: " << p.minimum_radius << "\n" << "\t"
          << "steering_ratio: " << p.steering_ratio << "\n";

      return os;
    }
  } Parameters;

private:
  StateStamped state_;
  Parameters params_;
public:
  VehicleKinematics(const VehicleKinematics::Parameters p) :
      params_(p)
  {
    state_.header.frame_id = params_.frame_id;
  }

  VehicleKinematics(const kut_ugv_vehicle::StateStamped& s, const VehicleKinematics::Parameters& p) :
      state_(s), params_(p)
  {
    state_.header.frame_id = params_.frame_id;
  }

  // Set new parameters. Resets the state variables
  inline void setParams(const VehicleKinematics::Parameters& p)
  {
    params_ = p;
    // reset the state as it will be inconsistent from now
    state_ = StateStamped();
    state_.header.frame_id = params_.frame_id;
  }

  inline const Parameters& getParams()
  {
    return params_;
  }

  inline const StateStamped& getState()
  {
    return state_;
  }

  inline void setState(const kut_ugv_vehicle::StateStamped& s)
  {
    state_ = s;
    ROS_WARN_COND(params_.frame_id != state_.header.frame_id,
                  "Setting new frame_id: '%s'", state_.header.frame_id.c_str());
  }

  // Reset all state variables to zero
  inline void resetState()
  {
    std::string frame_id;

    state_ = StateStamped();
    state_.header.frame_id = frame_id;

    ROS_WARN_COND(
        params_.frame_id != state_.header.frame_id,
        "state.frame_id is not equal to params.frame_id: '%s' vs '%s'", state_.header.frame_id.c_str(), params_.frame_id.c_str());
  }

  inline void updateState(const ros::Time& time_stamp, const tfScalar& velocity, const tfScalar& steering_angle,
                          const tfScalar& lateral_acc, const tfScalar& longitudal_acc, const tfScalar& yaw_rate,
                          const bool& emergency_state)
  {
    State s = state_.state;
    Parameters p = params_;
    ros::Duration dt = time_stamp - state_.header.stamp;

    // necessary for first iteration
//    if (dt == ros::Duration(0))
    if (state_.header.stamp == ros::Time(0))
    {
      state_.header.stamp = time_stamp;
      return;
    }

    // Distance from rear axle to c.r.
    tfScalar Lr = p.wheelbase_length - p.rotation_center;
    tfScalar R_1;

    s.lat_acc = lateral_acc;
    s.lon_acc = longitudal_acc;
    s.yaw_rate = yaw_rate;
    s.emergency = emergency_state;

    s.velocity = velocity;
    s.theta = steering_angle;
    s.beta = atan2(Lr * tan(s.theta), p.wheelbase_length);
    // 1 / R
    R_1 = cos(s.beta) * tan(s.theta) / p.wheelbase_length;
    // if steering angle is 0 then turning radius is R = inf;
    s.radius = 1 / R_1;
    // angular velocity around c.r.
    s.psi_dot = s.velocity * R_1;
    // integrate angular velocity
    s.psi += s.psi_dot * dt.toSec();
    s.x_dot = s.velocity * cos(s.psi + s.beta);
    s.y_dot = s.velocity * sin(s.psi + s.beta);
    // integrate Cartesian velocities
    s.x += s.x_dot * dt.toSec();
    s.y += s.y_dot * dt.toSec();

    // update state variable
    state_.header.stamp = time_stamp;
    state_.state = s;
  }

  static inline void stateMsgToOdometryMsg(const kut_ugv_vehicle::State& s, nav_msgs::Odometry& msg)
  {
    tf::Quaternion q;
    geometry_msgs::Quaternion q_msg;

//    msg.header.stamp = s.stamp;

    msg.pose.pose.position.x = s.x;
    msg.pose.pose.position.y = s.y;
    msg.pose.pose.position.z = 0.0;

    q.setRPY(0.0, 0.0, s.psi);
    tf::quaternionTFToMsg(q, q_msg);
    msg.pose.pose.orientation = q_msg;

    // TODO
    // msg.pose.covariance;

    msg.twist.twist.linear.x = s.x_dot;
    msg.twist.twist.linear.y = s.y_dot;
    msg.twist.twist.linear.z = 0.0;

    msg.twist.twist.angular.x = 0.0;
    msg.twist.twist.angular.y = 0.0;
    msg.twist.twist.angular.z = s.psi_dot;

    // TODO
    // msg.twist.covariance;
  }
  static inline void stateStampedMsgToOdometryMsg(const kut_ugv_vehicle::StateStamped& s,
                                                  nav_msgs::Odometry& msg,
                                                  const std::string& frame_id = std::string("odom"))
  {
    stateMsgToOdometryMsg(s.state, msg);
    msg.header = s.header;

    // Odometry child_frame_id is the frame attached to odometry sensor and
    // frame_id is the fixed frame where odometry data is zero
    msg.header.frame_id = frame_id;
    msg.child_frame_id = s.header.frame_id;
  }

};

#endif /* VEHICLE_KINEMATICS_HPP_ */
