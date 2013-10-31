#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

namespace mf = message_filters;
namespace it = image_transport;

class CameraSynchronizerNode
{
  typedef mf::sync_policies::ApproximateTime
      <sensor_msgs::Image,
      sensor_msgs::CameraInfo,
      sensor_msgs::Image,
      sensor_msgs::CameraInfo>
  ApproxSyncPolicy2;
  typedef mf::sync_policies::ApproximateTime
      <sensor_msgs::Image,
      sensor_msgs::CameraInfo,
      sensor_msgs::Image,
      sensor_msgs::CameraInfo,
      sensor_msgs::Image,
      sensor_msgs::CameraInfo>
  ApproxSyncPolicy3;
  typedef mf::Synchronizer<ApproxSyncPolicy2> SyncPolicy2;
  typedef mf::Synchronizer<ApproxSyncPolicy3> SyncPolicy3;
  typedef boost::shared_ptr<SyncPolicy2> SyncPolicy2Ptr;
  typedef boost::shared_ptr<SyncPolicy3> SyncPolicy3Ptr;

  ros::NodeHandlePtr node_;
  ros::NodeHandlePtr pnode_;

  // -- Synchronize up to 3 cameras
  boost::shared_ptr<it::ImageTransport> it_;
  it::Publisher image1_s_pub_;
  it::Publisher image2_s_pub_;
  it::Publisher image3_s_pub_;

  it::SubscriberFilter image1_sub_;
  mf::Subscriber<sensor_msgs::CameraInfo> info1_sub_;
  it::SubscriberFilter image2_sub_;
  mf::Subscriber<sensor_msgs::CameraInfo> info2_sub_;
  it::SubscriberFilter image3_sub_;
  mf::Subscriber<sensor_msgs::CameraInfo> info3_sub_;

  SyncPolicy2Ptr sync2_;
  SyncPolicy3Ptr sync3_;

  ros::Duration approximate_time_;
  int32_t queue_size_;

  std::string getTopicNamespace(std::string topic_name)
  {
    size_t pos = topic_name.find_last_of("/");
    return topic_name.substr(0, pos);
  }

public:
  CameraSynchronizerNode() :
      queue_size_(5)
  {
  }

  void callback2(const sensor_msgs::ImageConstPtr& image1,
                 const sensor_msgs::CameraInfoConstPtr info1,
                 const sensor_msgs::ImageConstPtr& image2,
                 const sensor_msgs::CameraInfoConstPtr info2
                 )
  {
    image1_s_pub_.publish(image1);
    image2_s_pub_.publish(image2);
  }

  void callback3(const sensor_msgs::ImageConstPtr& image1,
                 const sensor_msgs::CameraInfoConstPtr info1,
                 const sensor_msgs::ImageConstPtr& image2,
                 const sensor_msgs::CameraInfoConstPtr info2,
                 const sensor_msgs::ImageConstPtr& image3,
                 const sensor_msgs::CameraInfoConstPtr info3)
  {
    image1_s_pub_.publish(image1);
    image2_s_pub_.publish(image2);
    image3_s_pub_.publish(image3);
  }

  int init()
  {
    node_ = ros::NodeHandlePtr(new ros::NodeHandle);
    pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

    double approx_time;
    pnode_->param("approximate_time", approx_time, 0.01);
    approximate_time_ = ros::Duration(approx_time);

    pnode_->param("queue_size", queue_size_, 5);
    if (queue_size_ < 0)
    {
      ROS_ERROR("queue_size should be positive, falling back to default value (5)");
    }

    it_ = boost::shared_ptr<it::ImageTransport>(new it::ImageTransport(*node_));

    // If topic were not remapped do not subscribe
    if ((ros::names::remap("image1") != "image1") && (ros::names::remap("image2") != "image2"))
    {
      image1_sub_.subscribe(*it_, "image1", queue_size_);
      info1_sub_.subscribe(*node_, getTopicNamespace(image1_sub_.getTopic()) + "/camera_info", queue_size_);
      image1_s_pub_ = it_->advertise(image1_sub_.getTopic() + "_sync", queue_size_);
      image2_sub_.subscribe(*it_, "image2", queue_size_);
      info2_sub_.subscribe(*node_, getTopicNamespace(image2_sub_.getTopic()) + "/camera_info", queue_size_);
      image2_s_pub_ = it_->advertise(image2_sub_.getTopic() + "_sync", queue_size_);
      if (ros::names::remap("image3") != "image3")
      {
        image3_sub_.subscribe(*it_, "image3", queue_size_);
        info3_sub_.subscribe(*node_, getTopicNamespace(image3_sub_.getTopic()) + "/camera_info", queue_size_);
        image3_s_pub_ = it_->advertise(image3_sub_.getTopic() + "_sync", queue_size_);
        sync3_ = SyncPolicy3Ptr(new SyncPolicy3(ApproxSyncPolicy3(queue_size_),
                                                image1_sub_, info1_sub_,
                                                image2_sub_, info2_sub_,
                                                image3_sub_, info3_sub_));
//        sync3_->setMaxIntervalDuration(approximate_time_);
        sync3_->registerCallback(boost::bind(&CameraSynchronizerNode::callback3, this, _1, _2, _3, _4, _5, _6));
      }
      else
      {
        sync2_ = SyncPolicy2Ptr(new SyncPolicy2(ApproxSyncPolicy2(queue_size_),
                                                image1_sub_, info1_sub_,
                                                image2_sub_, info2_sub_));
//        sync2_->setMaxIntervalDuration(approximate_time_);
        sync2_->registerCallback(boost::bind(&CameraSynchronizerNode::callback2, this, _1, _2, _3, _4));
      }
    }
    else
    {
      ROS_FATAL("Topics image1 and image2 were not remapped");
      return 1;
    }

    return 0;
  }

  void publish()
  {
    ros::spin();
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_synchronizer");

  CameraSynchronizerNode ros_cam_sync;

  if(ros_cam_sync.init()) return 1;

  ros_cam_sync.publish();

  return 0;
}
