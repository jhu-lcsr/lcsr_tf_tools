
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
  transforms_.clear();
  std::string parent;

  for(auto frame_id=frame_ids_.begin(); 
      frame_id != frame_ids_.end(); 
      ++frame_id) {
    if(remote_listener_->getParent(*frame_id, time, parent)) {
      std::cerr<<"lookup "<<parent<<" -> "<<*frame_id<<std::endl;
      tf::StampedTransform transform;
      remote_listener_->lookupTransform(parent, *frame_id, time, transform);
      transforms_.push_back(transform);
    }
  }

  broadcaster_.sendTransform(transforms_);
}
