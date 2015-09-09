#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
import tf
from tf.transformations import quaternion_from_euler
import numpy as np

# def vec_to_quat(vec_msg):
#     rpy = np.radians((vec_msg.vector.x, vec_msg.vector.y, vec_msg.vector.z))
#     return quaternion_from_euler(*rpy)
#     # ps = PoseStamped()
#     # ps.header.stamp = rospy.Time.now()
#     # ps.header.frame_id = vec_msg.header.frame_id

class GlassPub(object):
    def __init__(self):
        self.quat = [0,0,0,1]
        self.glass_base_frame = '/glass_adjust'
        self.child_frame_id = 'glass'

        self.br = tf.TransformBroadcaster()
        self.pose_pub = rospy.Publisher('/android/pose', PoseStamped, queue_size=1)
        rospy.Subscriber('/android/pose_vec', Vector3Stamped, self.pose_vec_cb)
        t =rospy.Timer(rospy.Duration(0.05), self.pub)

    def pose_vec_cb(self, vec_msg):
        rpy = np.radians((vec_msg.vector.x, vec_msg.vector.y, vec_msg.vector.z))
        self.quat = quaternion_from_euler(*rpy)

    def pub(self, _):
        now = rospy.Time.now()
        self.br.sendTransform((0,0,0), self.quat, now, self.child_frame_id, self.glass_base_frame)

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.child_frame_id
        pose_msg.header.stamp = now
        pose_msg.pose.orientation.w = 1
        self.pose_pub.publish(pose_msg)


if __name__ == '__main__':
    rospy.init_node('vec2pose')
    GlassPub()
    rospy.spin()