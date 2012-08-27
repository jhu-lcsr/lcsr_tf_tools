#!/usr/bin/env python
import sys
import argparse

import roslib; roslib.load_manifest('lcsr_tf_tools')
import rospy
import tf

import PyKDL

import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs
import tf.msg

def main():
    # Parse arguments
    parser = argparse.ArgumentParser(description='Delete frames being broadcast by a multi_static_transform_publisher')
    parser.add_argument('--node-name', 
            metavar=('node_name'), 
            default=['multi_tf_pub'],
            type=str, nargs=1,
            help='The name of the multi publisher that should publish this transform. Default is \'multi_tf_pub\'')
    parser.add_argument('frame_ids', 
            metavar=('frame_id'), 
            type=str, nargs='+',
            help='The frame_ids of each frame to stop broadcasting.')

    args = parser.parse_args(rospy.myargv(argv=sys.argv))

    rospy.init_node('multi_static')

    topic_name = args.node_name[0]+'/del_frame'

    # Publish the transform
    pub = rospy.Publisher(topic_name, std_msgs.String, latch=True)
    for frame_id in args.frame_ids:
        pub.publish(std_msgs.String(frame_id))

    # Wait for sigint
    rospy.spin()

if __name__ == '__main__':
    main()
