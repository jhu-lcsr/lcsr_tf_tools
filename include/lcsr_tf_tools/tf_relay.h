#ifndef __LCSR_TF_TOOLS_TF_RELAY_H
#define __LCSR_TF_TOOLS_TF_RELAY_H

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/tfMessage.h>
#include <boost/unordered_set.hpp>

class TFRelay {
public:
  TFRelay(ros::NodeHandle nh);

  void sleep();

  void broadcast();

protected:

  void cb(const tf::tfMessageConstPtr &msg);

  ros::Rate broadcast_rate_;
  tf::tfMessage msg_;
  boost::unordered_set<std::string> frame_ids_;
  boost::unordered_set<std::string> filtered_frame_ids_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
};

#endif // ifndef __LCSR_TF_TOOLS_TF_RELAY_H
