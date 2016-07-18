#!/usr/bin/env python

"""
This script subscribes to one or more topics and re-maps time in the message's
header relative to the time at which the first message was published.
"""

from __future__ import print_function

import rospy
import rostopic
import tf
import tf2_msgs
import sys

class TimeShifter(object):
    def __init__(self):

        # Get topic class
        topic_class = None
        rospy.loginfo("Waiting for topic to become available...")
        while not rospy.is_shutdown() and topic_class == None:
            topic_class, topic, _ = rostopic.get_topic_class(rospy.resolve_name('in'))
            rospy.sleep(0.5)

        if hasattr(topic_class, 'header'):
            cb = self.header_cb
        elif topic_class == tf2_msgs.msg.TFMessage:
            cb = self.tf_cb
            self.frames = set(rospy.get_param('~frames'))
            self.frame_prefix = rospy.get_param('~frame_prefix')
        else:
            print("Message type {} doesn't have a header field.".format(topic_class))
            sys.exit(1)

        self.start_time = None
        self.actual_start_time = None
        self.time_offset = None

        self.sub = rospy.Subscriber('in', topic_class, cb)
        self.pub = rospy.Publisher('out', topic_class, queue_size=1)

    def shift(self, msg):
        if self.time_offset is None:
            start_time = rospy.Time.now()
            actual_start_time = msg.header.stamp
            self.time_offset = start_time - actual_start_time

        msg.header.stamp += self.time_offset

    def header_cb(self, msg):
        # Shift message
        self.shift(msg)
        self.pub.publish(msg)

    def tf_cb(self, msg):
        # Filter transforms
        msg.transforms = [tform for tform in msg.transforms if (tform.child_frame_id in self.frames)]

        # Shift transforms
        for tform in msg.transforms:
            tform.child_frame_id = '/'.join([self.frame_prefix, tform.child_frame_id])
            if tform.header.frame_id in self.frames:
                tform.header.frame_id = '/'.join([self.frame_prefix, tform.header.frame_id])
            self.shift(tform)

        self.pub.publish(msg)

def main():
    rospy.init_node('time_shifter')

    ts = TimeShifter()

    rospy.spin()

    return 0

if __name__ == '__main__':
    main()
