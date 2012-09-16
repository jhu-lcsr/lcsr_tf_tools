#!/usr/bin/env python

import fractions

import roslib; roslib.load_manifest('lcsr_tf_tools')
import threading
import rospy
import tf

import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs
import lcsr_tf_tools.msg as lcsr_tf_tools
import tf.msg

class MultiPublisher(object):
    def __init__(self):
        self.frames = dict()
        self.timers = dict()
        self.min_period = rospy.get_param('~min_period', 1.0)

        self.update_min_period()

        # Create some subscribers for adding / updating / removing frames
        self.add_sub = rospy.Subscriber('~set_frame', lcsr_tf_tools.StaticTransform, self.set_frame_cb)
        self.del_sub = rospy.Subscriber('~del_frame', std_msgs.String, self.del_frame_cb)

        # Create a raw tf publisher becuase pytf doesn't provide an api to publish
        # multipler transforms at once
        self.tf_pub = rospy.Publisher('/tf', tf.msg.tfMessage)

    def update_min_period(self):
        """ Update the minimum publish period."""
        #self.min_period = min([p.to_sec() for (f,p) in self.frames.values()])
        if len(self.frames) > 0:
            self.min_period = 1.0/1000.0*reduce(fractions.gcd, [1000.0*p.to_sec() for (f,p) in self.frames.values()])
            rospy.loginfo("Publish period: %g"%self.min_period)
        self.rate = rospy.Rate(1.0/self.min_period)
        # Reset the publish times for all frames to sync them up
        for (f,p) in self.frames.values():
            f.header.stamp = rospy.Time(0)


    def set_frame_cb(self,new_frame):
        """ Add or update a frame definition. """
        if new_frame.transform.child_frame_id in self.frames:
            rospy.loginfo("Received update to frame:" + str(new_frame.transform.child_frame_id))
        else:
            rospy.loginfo("Received new frame:" + str(new_frame.transform.child_frame_id))
        # Get the period from the timestamp
        period = rospy.Duration(new_frame.publish_period.to_sec())
        # Set the timestamp to THE BEGINNING OF TIME
        new_frame.transform.header.stamp = rospy.Time(0)
        # Store the new frame
        self.frames[new_frame.transform.child_frame_id] = (new_frame.transform, period)
        # Update the minimum sleeping period
        self.update_min_period()

    def del_frame_cb(self,doomed_frame):
        """ Delete a frame definition. """
        if doomed_frame.data in self.frames:
            del self.frames[doomed_frame.data]
            # Update the minimum sleeping period
            self.update_min_period()
            rospy.loginfo("Frame '%s' has been deleted." % (doomed_frame.data))
        else:
            rospy.logerr("Frame '%s' requested for deleteion, but it's not registered." % (doomed_frame.data))


    def broadcast_frames(self):
        """ Loop and publish frames at the appropriate periods while minimize bandiwdth use. """
        # Loop at a variable rate
        while not rospy.is_shutdown():
            # Get the current time
            now = rospy.Time.now()

            # Only publish frames that haven't been published at times after now
            frames_to_publish = [(f,p) for (f,p) in self.frames.values() if f.header.stamp <= now+rospy.Duration(0.020)] 

            # Update all the timestamps (and forward-date them)
            for (frame,period) in frames_to_publish:
                frame.header.stamp = now + period
            frames_to_publish = [f for (f,p) in frames_to_publish]

            #  Publish the list of frames
            if len(frames_to_publish) > 0:
                self.tf_pub.publish(tf.msg.tfMessage(frames_to_publish))
                rospy.logdebug("Publishing %d of %d frames: %s" % (
                    len(frames_to_publish),
                    len(frames_to_publish),
                    str([f.child_frame_id for f in frames_to_publish])))
            # Relax
            self.rate.sleep()

def main():
    rospy.init_node('multi_tf_pub')

    # Create the multi pub
    multi_pub = MultiPublisher()

    pub_thread = threading.Thread(target=multi_pub.broadcast_frames)
    pub_thread.start()

    rospy.spin()

    pub_thread.join()

if __name__ == '__main__':
    main()
