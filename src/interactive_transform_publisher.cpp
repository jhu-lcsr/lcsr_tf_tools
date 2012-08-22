/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdio>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <stdexcept>
#include <sstream>


class TransformSender {
public:
  TransformSender(
      tf::StampedTransform transform) :
    transform_(transform)
  { 
    if(transform.child_frame_id_ == transform.frame_id_) {
      std::ostringstream oss;
      oss<<"A transform's child frame_id (\""<<transform.child_frame_id_<<"\") must be different than its frame_id (\""<<transform.frame_id_<<"\").";
      ROS_FATAL_STREAM(oss.str());
      throw std::runtime_error(oss.str());
    }
  }

  ~TransformSender() { }

  //A pointer to the rosTFServer class
  tf::TransformBroadcaster broadcaster;

  void interactive_feedback_cb(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
  {
    // Update the transform
    transform_.setData(tf::Transform(
          tf::Quaternion(feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w),
          tf::Vector3(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z)));
  }

  // A function to call to send data periodically
  void send (ros::Time time) {
    ROS_DEBUG_STREAM("Sending transform from "<<transform_.frame_id_<<" to "<<transform_.child_frame_id_);
    transform_.stamp_ = time;
    broadcaster.sendTransform(transform_);
  };

private:
  tf::StampedTransform transform_;

};

int main(int argc, char ** argv)
{
  //Initialize ROS
  ros::init(argc, argv,"static_transform_publisher", ros::init_options::AnonymousName);

  ros::NodeHandle nh;
  tf::StampedTransform transform;
  tf::Quaternion orientation;
  ros::Rate rate(1.0);

  switch(argc) {
    case 10: // x y z yaw pitch roll frame_id child_frame_id period
      orientation.setRPY(atof(argv[6]), atof(argv[5]), atof(argv[4]));

      transform = tf::StampedTransform(
          tf::Transform(
            orientation,
            tf::Vector3(atof(argv[1]), atof(argv[2]), atof(argv[3]))),
          ros::Time(),
          argv[7], argv[8]);

      rate = ros::Rate(1000.0/atof(argv[9]));
      break;

    case 11: // x y z qx qy qz frame_id child_frame_id period
      transform = tf::StampedTransform(
          tf::Transform(
            tf::Quaternion(atof(argv[4]), atof(argv[5]), atof(argv[6]), atof(argv[7])),
              tf::Vector3(atof(argv[1]), atof(argv[2]), atof(argv[3]))),
            ros::Time(),
            argv[8], argv[9]);

      rate = ros::Rate(1000.0/atof(argv[10]));
      break;

    default:
      printf("A command line utility for manually sending a transform.\n");
      printf("It will periodicaly republish the given transform. \n");
      printf("Usage: static_transform_publisher x y z  yaw(z) pitch(y) roll(x)  frame_id child_frame_id  period(milliseconds) \n");
      printf("OR \n");
      printf("Usage: static_transform_publisher x y z  qx qy qz qw  frame_id child_frame_id  period(milliseconds) \n");
      printf("\nThis transform is the transform of the coordinate frame from frame_id into the coordinate frame \n");
      printf("of the child_frame_id.  \n");
      ROS_ERROR("static_transform_publisher exited due to not having the right number of arguments");
      return -1;
  };

  // Create sender
  TransformSender tf_sender(transform);

  // Create interactive marker server
  interactive_markers::InteractiveMarkerServer server("interactive_transform_publisher");

  // Create an interactive marker 
  using namespace visualization_msgs;
  InteractiveMarker int_marker;
  int_marker.header.frame_id = transform.frame_id_;
  int_marker.name = transform.child_frame_id_;
  int_marker.description = "Interactive Transform Publisher";
  int_marker.scale = 0.2;

  // Create a sphere marker
  Marker sphere_marker;
  sphere_marker.type = Marker::SPHERE;
  sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z = 0.05;
  sphere_marker.color.r = sphere_marker.color.g = sphere_marker.color.b = 0.5;
  sphere_marker.color.a = 1.0;

  // create a non-interactive control which contains the box
  InteractiveMarkerControl tf_control;
  tf_control.always_visible = false;
  tf_control.markers.push_back( sphere_marker );

  // add the control to the interactive marker
  int_marker.controls.push_back( tf_control );

  InteractiveMarkerControl control;
  control.orientation.w = 1;

  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server.insert(int_marker, boost::bind(&TransformSender::interactive_feedback_cb, &tf_sender, _1));

  // 'commit' changes and send to all clients
  server.applyChanges();

  // Loop
  while(ros::ok()) {
    //Future dating to allow slower sending w/o timeout
    tf_sender.send(ros::Time::now() + rate.expectedCycleTime());
    ros::spinOnce();
    rate.sleep();
  }

  return 0;

};

