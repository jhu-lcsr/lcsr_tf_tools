#!/usr/bin/env python

import roslib; roslib.load_manifest('lcsr_tf_tools')
import rospy
import tf
import geometry_msgs.msg as geometry_msgs
import threading
import math
import PyKDL
import copy

def multiply_quat(q0,q1):
    [x0,y0,z0,w0] = q0
    [x1,y1,z1,w1] = q1
    return ( (w0*x1 + x0*w1 + y0*z1 - z0*y1),
             (w0*y1 - x0*z1 + y0*w1 + z0*x1),
             (w0*z1 + x0*y1 - y0*x1 + z0*w1),
             (w0*w1 - x0*x1 - y0*y1 - z0*z1) )

class TwistFrameIntegrator(object):
    def __init__(self):
        # Get the node params
        self.linear_multiplier = rospy.get_param('~linear_multiplier')
        self.angular_multiplier = rospy.get_param('~angular_multiplier')
        self.broadcast_rate = rospy.get_param('~broadcast_rate')
        self.body_fixed = rospy.get_param('~body_fixed')

        # Initialize the frame we're going to publish
        xyzw = [float(f) for f in rospy.get_param('~xyzw','0 0 0 1').split()]
        xyzw_offset = [float(f) for f in rospy.get_param('~xyzw_offset','0 0 0 1').split()]
        xyz = [float(f) for f in rospy.get_param('~xyz','0 0 0').split()]
        rospy.logwarn("xyzw: " + str(xyzw))
        rospy.logwarn("xyz: " + str(xyz))
        self.transform = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(*xyzw)*PyKDL.Rotation.Quaternion(*xyzw_offset),
            PyKDL.Vector(*xyz))
        self.transform_out = PyKDL.Frame()
        self.rotation = self.transform.M.GetQuaternion()
        self.time = rospy.Time.now()
        self.frame_id = rospy.get_param('~frame_id')
        self.child_frame_id = rospy.get_param('~child_frame_id')

        # Synchronization
        self.broadcast_lock = threading.Lock()

        # Initialze TF structures
        self.broadcaster = tf.TransformBroadcaster()

        # Subscribe to twist inputs
        self.twist_sub = rospy.Subscriber(
                "twist", geometry_msgs.Twist,
                self.twist_cb)
        self.twiststamped_sub = rospy.Subscriber(
                "twist_stamped", geometry_msgs.TwistStamped,
                self.twiststamped_cb)

        # Broadcast the frame at a given rate
        self.broadcast_thread = threading.Thread(target=self.broadcast_loop)
        self.broadcast_thread.start()

    def broadcast_loop(self):
        """
        Broadcast the integrated TF frame at a fixed rate.
        """
        rate = rospy.Rate(self.broadcast_rate)
        while not rospy.is_shutdown():
            # Acquire broadcast lock
            self.broadcast_lock.acquire()
            # Broadcast the frame
            try:
                self.broadcaster.sendTransform(
                        (self.transform_out.p.x(), self.transform_out.p.y(), self.transform_out.p.z()),
                        self.transform_out.M.GetQuaternion(),
                        rospy.Time.now(),
                        self.child_frame_id,
                        self.frame_id)
            except Exception as ex:
                rospy.logerr("Failed to broadcast transform: "+str(ex))
            # Release broadcast lock
            self.broadcast_lock.release()
            # Don't spam TF
            rate.sleep()

    def twiststamped_cb(self,msg):
        """
        This callback pulls the twist out of a TwistStamped message. For now,
        it disregards the origin frame.
        """
        twist_cb(msg.twist)

    def twist_cb(self,msg):
        """
        This callback integrates the linear and angular velocities into a TF
        frame, and then broadcasts the frame.
        """

        try:
            # Acquire broadcast lock
            self.broadcast_lock.acquire()

            # Convert angular rotation vector to quaternion
            q_velocity = (
                    msg.angular.x * math.sin(self.angular_multiplier/2),
                    msg.angular.y * math.sin(self.angular_multiplier/2),
                    msg.angular.z * math.sin(self.angular_multiplier/2),
                    math.cos(self.angular_multiplier/2))

            q_velocity_norm = math.sqrt(math.pow(q_velocity[0],2) + math.pow(q_velocity[1],2) + math.pow(q_velocity[2],2) + math.pow(q_velocity[3],2))
            q_velocity = [i/q_velocity_norm for i in q_velocity]

            if self.body_fixed:
                # Integrate linear velocity in local frame
                self.transform.p += self.transform.M*(self.linear_multiplier*PyKDL.Vector(msg.linear.x,msg.linear.y,msg.linear.z))
                # Integrate angular velocity in local frame
                self.rotation = multiply_quat(self.rotation,q_velocity)
                self.transform.M = PyKDL.Rotation.Quaternion(*self.rotation)
                
                # Copy this transform
                self.transform_out = PyKDL.Frame(self.transform)
            else:
                # Integrate linear velocity
                self.transform.p += self.linear_multiplier*PyKDL.Vector(msg.linear.x,msg.linear.y,msg.linear.z)
                # Integrate angular velocity
                self.rotation = multiply_quat(q_velocity,self.rotation)
                self.transform.M = PyKDL.Rotation.Quaternion(*self.rotation)

                # Invert the transform to get parent-frame relative transform
                self.transform_out = self.transform

            self.transform_out.M.DoRotZ(-math.pi/2)
            self.transform_out.M.DoRotX(-math.pi/2)



        finally:
            # Release broadcast lock
            self.broadcast_lock.release()

def main():
    rospy.init_node('twist_frame_integrator')
    
    twist_frame_integrator = TwistFrameIntegrator()

    rospy.loginfo("Spinning...")
    rospy.spin()
    rospy.loginfo("Done.")


if __name__ == '__main__':
    main()
