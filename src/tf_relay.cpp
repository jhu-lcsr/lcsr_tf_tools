
#include <ros/ros.h>
#include <lcsr_tf_tools/tf_relay.h>

TFRelay::TFRelay(ros::NodeHandle nh) :
  delay_(0.0),
  broadcast_rate_(25.0),
  buffered_transforms_(TransformStampedCmp(true))
{

  ros::NodeHandle nhp("~");

  // Get desired broadcast rate
  double broadcast_rate;
  if(nhp.getParam("broadcast_rate", broadcast_rate)) {
    broadcast_rate_ = ros::Rate(broadcast_rate);
  }

  // Get relay delay
  double delay;
  if(nhp.getParam("delay", delay)) {
    delay_ = ros::Duration(delay);
  }

  // Get frame ids to relay
  std::vector<std::string> frame_ids;
  nhp.getParam("frame_ids", frame_ids);

  for(std::vector<std::string>::const_iterator it=frame_ids.begin(); it != frame_ids.end(); ++it) {
    frame_ids_.insert(*it);
  }

  std::vector<std::string> filtered_frame_ids;
  nhp.getParam("filtered_frame_ids", filtered_frame_ids);
  for(std::vector<std::string>::const_iterator it=filtered_frame_ids.begin(); it != filtered_frame_ids.end(); ++it) {
    filtered_frame_ids_.insert(*it);
  }

  pub_ = nh.advertise<tf::tfMessage>("tf_out", 50);

  bool use_udp = false;
  nhp.getParam("udp", use_udp);
  ros::TransportHints hints;
  if(use_udp) {
    hints = hints.udp();
  }
  tf_sub_ = nh.subscribe("tf_in", 300, &TFRelay::cb, this, hints);
  set_delay_sub_ = nhp.subscribe("set_delay", 1, &TFRelay::set_delay_cb, this);
}

void TFRelay::sleep()
{
  broadcast_rate_.sleep();
}

void TFRelay::broadcast()
{
  // Add all buffereed transforms that are ready to be broadcast
  while(not buffered_transforms_.empty() and buffered_transforms_.top().header.stamp + delay_ <= ros::Time::now()) {
    msg_.transforms.push_back(buffered_transforms_.top());
    buffered_transforms_.pop();
  }
  pub_.publish(msg_);
  msg_.transforms.clear();
}

void TFRelay::set_delay_cb(const std_msgs::Float64 &msg)
{
  delay_ = ros::Duration(msg.data);
}


void TFRelay::cb(const tf::tfMessageConstPtr &msg)
{
  // Filter the messages based on child_frame_id
  for(std::vector<geometry_msgs::TransformStamped>::const_iterator it=msg->transforms.begin();
      it!=msg->transforms.end();
      ++it)
  {
    std::string frame_id = it->child_frame_id;
    if(frame_id[0] != '/') {
      frame_id.insert(0, "/");
    }

    size_t sep_index = 0;
    while(true) {
      // Look for the next separator
      sep_index = frame_id.find("/",sep_index+1);

      // Look up the partial frame id in the hashmap
      std::string sub_frame_id = frame_id.substr(0,sep_index);
      if(frame_ids_.find(sub_frame_id) != frame_ids_.end() and filtered_frame_ids_.find(frame_id) == filtered_frame_ids_.end()) {
        // Add this transform to the buffered message if it's ready to be relayed, otherwise, add it to the queue
        buffered_transforms_.push(*it);
        ROS_DEBUG_STREAM(frame_id<<" relayed");
        break;
      } else {
        ROS_DEBUG_STREAM(frame_id<<" not relayed");
      }

      // Break once we've compared the whole string
      if(sep_index == std::string::npos) {
        break;
      }
    }
  }
}
