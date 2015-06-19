#ifndef __LCSR_TF_TOOLS_TF_RELAY_H
#define __LCSR_TF_TOOLS_TF_RELAY_H

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/tfMessage.h>
#include <boost/unordered_set.hpp>
#include <queue>

class TransformStampedCmp {
public:
  TransformStampedCmp(const bool& revparam=false) :reverse(revparam) {}
  bool operator() (const geometry_msgs::TransformStamped& lhs, const geometry_msgs::TransformStamped&rhs) const {
    if (reverse) {
      return (lhs.header.stamp>rhs.header.stamp);
    } else {
      return (lhs.header.stamp<rhs.header.stamp);
    }
  }
private:
  bool reverse;
};


class TFRelay {
public:
  TFRelay(ros::NodeHandle nh);

  void sleep();

  void broadcast();

protected:

  void cb(const tf::tfMessageConstPtr &msg);

  ros::Duration delay_;
  ros::Rate broadcast_rate_;
  tf::tfMessage msg_;
  std::priority_queue<
    geometry_msgs::TransformStamped,
    std::vector<geometry_msgs::TransformStamped>,
    TransformStampedCmp>
      buffered_transforms_;
  boost::unordered_set<std::string> frame_ids_;
  boost::unordered_set<std::string> filtered_frame_ids_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
};

#endif // ifndef __LCSR_TF_TOOLS_TF_RELAY_H
