// ROS
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <algorithm>
#include <functional>
#include <cstdlib>

#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include <kut_ugv_msgs/MotionCommandStamped.h>
#include "vehicle_kinematics.hpp"

using boost::asio::ip::udp;

// Default publish rate
const int32_t PUBLISH_RATE = 100; // 100Hz

class CompactRIOCommunicationNode
{
  enum
  {
    max_length = 1024
  };
  char data_[max_length];

public:
  int32_t publish_rate_;
  int32_t socket_timeout_;

  ros::NodeHandlePtr node_;
  // Private node for parameters
  ros::NodeHandlePtr pnode_;

  ros::Publisher odom_pub_;
  ros::Publisher state_pub_;
  // subscribe to control message
  ros::Subscriber sub_;

  std::string tf_prefix_;

  // Odometry fixed frame
  std::string fixed_frame_id_;
  // CompactRIO IP address and port
  std::string crio_ip_;
  std::string crio_cmd_port_;
  std::string crio_state_port_;

  geometry_msgs::Pose pose_;
  geometry_msgs::Twist twist_;

  // Transform for odometry
  geometry_msgs::TransformStamped odom_tf_;
  boost::shared_ptr<tf::TransformBroadcaster> tf_br_;
  nav_msgs::Odometry odom_msg_;

  boost::asio::io_service io_service_;
  udp::endpoint send_ep_;
  udp::endpoint receive_ep_;
  udp::socket socket_;
  boost::asio::deadline_timer deadline_;
  boost::posix_time::seconds timeout_;
  boost::system::error_code error_;

  boost::shared_ptr<VehicleKinematics> kin_;

  boost::mutex mutex_;

  bool publish_odom_tf_;

  CompactRIOCommunicationNode() :
      publish_rate_(100), socket_timeout_(10), socket_(io_service_), deadline_(io_service_),
      timeout_(socket_timeout_), publish_odom_tf_(true)
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
    pnode_->param("publish_rate", publish_rate_, PUBLISH_RATE);

    pnode_->param("fixed_frame_id", fixed_frame_id_, std::string("odom"));
    fixed_frame_id_ = tf::resolve(tf_prefix_, fixed_frame_id_);

    pnode_->param("publish_odom_tf", publish_odom_tf_, true);

    pnode_->param("crio/ip", crio_ip_, std::string("127.0.0.1"));
    pnode_->param("crio/command_port", crio_cmd_port_, std::string("39000"));
    pnode_->param("crio/state_port", crio_state_port_, std::string("39001"));
    pnode_->param("crio/socket_timeout", socket_timeout_, 10);

    VehicleKinematics::Parameters kin_params;
    tfScalar minimum_radius_outer;

    if (!pnode_->getParam("kinematics/frame_id", kin_params.frame_id))
    {
      ROS_WARN_STREAM(pnode_->getNamespace() << "/kinematics/frame_id parameter is not set");
    }
    else
    {
      kin_params.frame_id = tf::resolve(tf_prefix_, kin_params.frame_id);
    }

    if (!pnode_->getParam("kinematics/wheelbase", kin_params.wheelbase_length))
    {
      ROS_FATAL_STREAM(pnode_->getNamespace() << "/kinematics/wheelbase parameter is not set");
      return -1;
    }
    if (!pnode_->getParam("kinematics/track", kin_params.track_width))
    {
      ROS_FATAL_STREAM( pnode_->getNamespace() << "/kinematics/track parameter is not set");
      return -1;
    }
    if (!pnode_->getParam("kinematics/rotation_center", kin_params.rotation_center))
    {
      ROS_WARN_STREAM(
          pnode_->getNamespace()
          << "/kinematics/rotation_center parameter is not set. Using default: wheelbase/2 = "
          << kin_params.wheelbase_length / 2);
      kin_params.rotation_center = kin_params.wheelbase_length / 2;
    }
    if (!pnode_->getParam("kinematics/minimum_radius_outer", minimum_radius_outer))
    {
      ROS_FATAL_STREAM(pnode_->getNamespace() << "/kinematics/minimum_radius_outer parameter is not set");
      return -1;
    }
    else
    {
      kin_params.minimum_radius = minimum_radius_outer - kin_params.track_width / 2;
    }
    if (!pnode_->getParam("kinematics/steering_ratio", kin_params.steering_ratio))
    {
      ROS_FATAL_STREAM(pnode_->getNamespace() << "/kinematics/steering_ratio parameter is not set");
      return -1;
    }

//    kin_(2.7, 1.626, 1.35);
    kin_ = boost::make_shared<VehicleKinematics>(kin_params);

    sub_ = node_->subscribe<kut_ugv_msgs::MotionCommandStamped>("motion_command", 200,
                                                                &CompactRIOCommunicationNode::motionCommand, this);
    odom_pub_ = node_->advertise<nav_msgs::Odometry>("odom", 200);
    state_pub_ = node_->advertise<kut_ugv_vehicle::StateStamped>("state", 200);

    tf_br_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster);

    timeout_ = boost::posix_time::seconds(socket_timeout_);
    send_ep_ = udp::endpoint(boost::asio::ip::address::from_string(crio_ip_), std::atoi(crio_cmd_port_.c_str()));
    receive_ep_ = udp::endpoint(udp::v4(), std::atoi(crio_state_port_.c_str()));

    socket_.open(udp::v4());

    deadline_.expires_at(boost::posix_time::pos_infin);
    this->deadlineCallback(deadline_, socket_);

    return 0;
  }

  void deadlineCallback(boost::asio::deadline_timer& t, udp::socket& s)
  {
    if (t.expires_at() <= boost::asio::deadline_timer::traits_type::now())
    {
      s.cancel();
      t.expires_at(boost::posix_time::pos_infin);
    }
    t.async_wait(boost::bind(&CompactRIOCommunicationNode::deadlineCallback, this, boost::ref(t), boost::ref(s)));
  }

  void handleRead(const boost::system::error_code& ec, std::size_t ln)
  {
    ros::Time cur_time = ros::Time::now();

    error_ = ec;
    if (!socket_.is_open())
      return;

    if (!ec)
    {
      std::string msg(data_);
      std::istringstream ss;

//      ss.unsetf(std::ios::floatfield);
//      // set %7.7f
//      ss.precision(7);
//      ss.width(7);

      if (!msg.empty())
      {
        boost::mutex::scoped_lock lock(mutex_);

        double velocity, heading, lat_acc, lon_acc, yaw_rate;
        // Emergency: 1 - ok, 0 - fault
        bool emergency_state;

        ss.str(msg);
        ss >> heading >> velocity >> lat_acc >> lon_acc >> yaw_rate >> emergency_state;

        kin_->updateState(cur_time, velocity, heading, lat_acc, lon_acc, yaw_rate, emergency_state);

        nav_msgs::Odometry msg;
        VehicleKinematics::stateStampedMsgToOdometryMsg(kin_->getState(), msg);

//        ROS_INFO_STREAM(kin_.getState());

//        msg.header.stamp = cur_time;
//        msg.header.frame_id = "base_link";
//        msg.child_frame_id = "odom";

        // Resolve frame names
        msg.header.frame_id = tf::resolve(tf_prefix_, msg.header.frame_id);
        msg.child_frame_id = tf::resolve(tf_prefix_, msg.child_frame_id);

        odom_tf_.header = msg.header;
        odom_tf_.child_frame_id = msg.child_frame_id;

        odom_tf_.transform.translation.x = msg.pose.pose.position.x;
        odom_tf_.transform.translation.y = msg.pose.pose.position.y;
        odom_tf_.transform.translation.z = msg.pose.pose.position.z;

        odom_tf_.transform.rotation = msg.pose.pose.orientation;

        odom_msg_ = msg;
      }
      else
      {
        ROS_WARN_STREAM("Empty message");
      }
    }
  }

  void motionCommand(const kut_ugv_msgs::MotionCommandStampedConstPtr& cmd)
  {
    std::ostringstream ss;

    ss.unsetf(std::ios::floatfield);
    // set %7.7f
    ss.precision(7);
    ss.width(7);

    ss << cmd->lateral_offset
        << " " << 0.0 // tangential angle
        << " " << 0.0 // heading angle
        << " " << cmd->curvature
        << " " << cmd->heading_delta
        << " " << cmd->velocity_limit
        << " " << 0.0 // current x
        << " " << 0.0 // current y
        << " " << 0.0 // waypoint x
        << " " << 0.0 // waypoint y
        << " " << 0.0 // second lateral offset
        << "\n";

    try
    {
      socket_.send_to(boost::asio::buffer(std::string(ss.str())), send_ep_);
    }
    catch (std::exception& e)
    {
      ROS_ERROR_STREAM(e.what());
    }
  }

  void readCRIO()
  {
    udp::socket socket2(io_service_, receive_ep_);
    while (node_->ok())
    {
      try
      {
        deadline_.expires_from_now(timeout_);
        error_ = boost::asio::error::would_block;

        socket2.async_receive_from(boost::asio::buffer(data_, max_length), receive_ep_,
                                   boost::bind(&CompactRIOCommunicationNode::handleRead, this, _1, _2));

        do
        {
          io_service_.run_one();
        } while (error_ == boost::asio::error::would_block);

        if (error_)
        {
          if (error_ == boost::asio::error::operation_aborted)
          {
            ROS_WARN("Socket receive timed out");
          }
          else
          {
            throw boost::system::system_error(error_);
          }
        }
      }
      catch (std::exception& e)
      {
        ROS_ERROR_STREAM(e.what());
      }
    }
  }

  void publish()
  {
    ros::Rate loop_rate(publish_rate_);
    ros::AsyncSpinner spinner(2);
    // handle communication with cRIO in separate thread
    boost::thread crio_read_thread(&CompactRIOCommunicationNode::readCRIO, this);

    spinner.start();

    while (node_->ok())
    {
      // lock variables from being modified
      // by cRIO communication thread
      {
        boost::mutex::scoped_lock lock(mutex_);

        if(publish_odom_tf_) tf_br_->sendTransform(odom_tf_);

        odom_pub_.publish(odom_msg_);
        state_pub_.publish(kin_->getState());
      }

      //    ros::spinOnce();
      loop_rate.sleep();
    }

    crio_read_thread.join();
  }
};

int main(int argc, char** argv)
{
  //-- Init ROS node
  ros::init(argc, argv, "crio_comm");
  CompactRIOCommunicationNode crio_ros;

  if(!crio_ros.init()) crio_ros.publish();
  else return -1;

  return 0;
}
