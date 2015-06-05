
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/tfMessage.h>


#include <boost/unordered_set.hpp>


class TFRelay {
public:
  TFRelay(ros::NodeHandle nh) :
    broadcast_rate_(50.0)
  {

    ros::NodeHandle nhp("~");

    double broadcast_rate;
    nhp.getParam("broadcast_rate", broadcast_rate);
    broadcast_rate_ = ros::Rate(broadcast_rate);

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
    sub_ = nh.subscribe("tf_in", 100, &TFRelay::cb, this);
  }

  void sleep() {
    broadcast_rate_.sleep();
  }

  void broadcast() {
    pub_.publish(msg_);
    msg_.transforms.clear();
  }

protected:

  void cb(const tf::tfMessageConstPtr &msg)
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
          // Add this transform to the buffered message
          msg_.transforms.push_back(*it);
          ROS_INFO_STREAM(frame_id<<" relayed");
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

  ros::Rate broadcast_rate_;
  tf::tfMessage msg_;
  boost::unordered_set<std::string> frame_ids_;
  boost::unordered_set<std::string> filtered_frame_ids_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
};


int main(int argc, char** argv) {

  ros::init(argc, argv, "tf_relay");

  ros::NodeHandle nh;

  TFRelay relay(nh);

  double publish_rate;

  while(ros::ok()) {
    relay.broadcast();
    relay.sleep();
    ros::spinOnce();
  }

  return 0;
}
