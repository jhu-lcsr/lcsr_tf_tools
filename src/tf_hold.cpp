
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
  std::map<std::string, std::string> remote_tf_remapping;
  remote_tf_remapping["/tf"] = "/remote/tf";
  remote_nh_ = ros::NodeHandle(nh, "/remote", remote_tf_remapping);

  // Create a tf listener for remote frames only
  remote_listener_ = boost::make_shared<tf::TransformListener>(remote_nh_, max_cache_time);
}

void TFHold::broadcast(ros::Time time)
{
  remote_listener_->getFrameStrings(frame_ids_);
  transforms_.resize(frame_ids_.size());
  transform_msgs_.resize(frame_ids_.size());
  std::string parent;

  auto frame_id=frame_ids_.begin();
  auto transform=transforms_.begin();
  auto transform_msg=transform_msgs_.begin();

  for(;frame_id != frame_ids_.end(); ++frame_id, ++transform, ++transform_msg) {
    remote_listener_->getParent(*frame_id, time, parent);
    remote_listener_->lookupTransform(parent, *frame_id, time, *transform);
    tf::transformStampedTFToMsg(*transform, *transform_msg);
  }

  broadcaster_.sendTransform(transforms_);
}
