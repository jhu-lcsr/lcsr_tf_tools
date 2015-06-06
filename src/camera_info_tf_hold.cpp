
#include <ros/ros.h>
#include <lcsr_tf_tools/tf_hold.h>
#include <sensor_msgs/CameraInfo.h>

class CameraInfoTFHold : public lcsr_tf_tools::TFHold {
public:

  CameraInfoTFHold(ros::NodeHandle nh, ros::Duration max_cache_time)
    : lcsr_tf_tools::TFHold(nh, max_cache_time)
  { }

  void subscribe() {
    info_sub_ = nh_.subscribe("camera_info", 30, &CameraInfoTFHold::info_cb, this);
  }

protected:

  void info_cb(const sensor_msgs::CameraInfoConstPtr &camera_info) {
    this->broadcast(camera_info->header.stamp);
  }

  ros::Subscriber info_sub_;
};

int main(int argc, char** argv) {

  ros::init(argc, argv, "camera_info_tf_hold");

  ros::NodeHandle nh;

  CameraInfoTFHold citf(nh, ros::Duration(30.0));
  citf.subscribe();

  ros::spin();

  return 0;
}
