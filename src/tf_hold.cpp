
#include <lcsr_tf_tools/tf_hold.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <boost/make_shared.hpp>

using namespace lcsr_tf_tools;

TFHold::TFHold(ros::NodeHandle nh, ros::Duration max_cache_time) :
  nh_(nh)
{
  // Construct tf "broadcaster" publisher
  tf_pub_ = nh.advertise<tf::tfMessage>("tf_out", 10);

  // Construct tf remapping & node handle
  // This is sort of awkward because we want to be able to remap `tf_in` to `/tf`
  // for the transform listener
  std::map<std::string, std::string> tf_remapping;
  tf_remapping["/tf"] = nh.resolveName("tf_in");
  ros::NodeHandle remapped_nh = ros::NodeHandle(nh, nh.getNamespace(), tf_remapping);

  // Create a tf listener for remote frames only
  remote_listener_ = boost::make_shared<tf::TransformListener>(remapped_nh, max_cache_time);
}

void TFHold::broadcast(ros::Time time)
{
  tf_msg_.transforms.clear();
  remote_listener_->getFrameStrings(frame_ids_);
  std::string parent;

  // Temporaries
  tf::StampedTransform transform;
  geometry_msgs::TransformStamped transform_msg;

  // Get the current time
  ros::Time time_now = ros::Time::now();

  for(auto &frame_id: frame_ids_) {
    parent = "NO_PARENT";
    if(remote_listener_->getParent(frame_id, time, parent)) {
      remote_listener_->lookupTransform(parent, frame_id, time, transform);
      transformStampedTFToMsg(transform, transform_msg);
      transform_msg.header.stamp = time_now;
      tf_msg_.transforms.push_back(transform_msg);
    } else {
      ROS_DEBUG_STREAM("Couldn't lookup \""<<frame_id<<"\" at time "<<time<<" got parent "<<parent);
    }
  }

  // publish the transform
  tf_pub_.publish(tf_msg_);
}
