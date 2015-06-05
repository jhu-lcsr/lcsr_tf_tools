#ifndef __LCSR_TF_TOOLS_TF_HOLD
#define __LCSR_TF_TOOLS_TF_HOLD

#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/CameraInfo.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <boost/shared_ptr.hpp>

namespace lcsr_tf_tools {
  class TFHold {
  public:
    TFHold(ros::NodeHandle nh, ros::Duration max_cache_time = ros::Duration(30.0));

    // Broadcast all frames at this time
    void broadcast(ros::Time time);

  protected:
    ros::NodeHandle nh_;

    boost::shared_ptr<tf::TransformListener> remote_listener_;
    tf::TransformBroadcaster broadcaster_;

    std::vector<std::string> frame_ids_;
    std::vector<tf::StampedTransform> transforms_;
    std::vector<geometry_msgs::TransformStamped> transform_msgs_;
  };
}

#endif // ifndef __LCSR_TF_TOOLS_TF_HOLD
