
#include <lcsr_tf_tools/tf_hold.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <boost/make_shared.hpp>

using namespace lcsr_tf_tools;

TFHold::TFHold(ros::NodeHandle nh, ros::Duration max_cache_time) :
  nh_(nh)
{
  // Construct tf remapping & node handle
  std::map<std::string, std::string> tf_remapping;
  tf_remapping["/tf"] = "tf";
  ros::NodeHandle remapped_nh = ros::NodeHandle(nh, nh.getNamespace(), tf_remapping);

  // Create a tf listener for remote frames only
  remote_listener_ = boost::make_shared<tf::TransformListener>(remapped_nh, max_cache_time);
}

void TFHold::broadcast(ros::Time time)
{
  remote_listener_->getFrameStrings(frame_ids_);
  transforms_.clear();
  std::string parent;

  for(auto &frame_id: frame_ids_) {
    parent = "NO_PARENT";
    if(remote_listener_->getParent(frame_id, time, parent)) {
      tf::StampedTransform transform;
      remote_listener_->lookupTransform(parent, frame_id, time, transform);
      transforms_.push_back(transform);
    } else {
      ROS_WARN_STREAM("Couldn't lookup \""<<frame_id<<"\" at time "<<time<<" got parent "<<parent);
    }
  }

  broadcaster_.sendTransform(transforms_);
}
