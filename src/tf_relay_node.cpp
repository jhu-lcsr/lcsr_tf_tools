
#include <ros/ros.h>
#include <lcsr_tf_tools/tf_relay.h>


int main(int argc, char** argv) {

  ros::init(argc, argv, "tf_relay");

  ros::NodeHandle nh;

  TFRelay relay(nh);

  while(ros::ok()) {
    relay.broadcast();
    relay.sleep();
    ros::spinOnce();
  }

  return 0;
}
