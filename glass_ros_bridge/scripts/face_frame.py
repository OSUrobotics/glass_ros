#!/usr/bin/env python

import roslib; roslib.load_manifest('glass_ros_bridge')
import rospy

from people_msgs.msg import PositionMeasurement
from geometry_msgs.msg import PointStamped, PoseStamped

import tf
from tf.transformations import quaternion_from_euler

from math import sqrt, atan2, pi


class HeadFrame:
    def __init__(self):
        self.found_face = False

        # Set up a tf broadcaster
        self.broadcaster = tf.TransformBroadcaster()

        # Set up a tf listener
        self.listener = tf.TransformListener()

        # Set up the face detector callback
        self.sub = rospy.Subscriber('/face_detector/people_tracker_measurements',
                                    PositionMeasurement,
                                    self.face_callback)

    def face_callback(self, msg):
        if not self.found_face:
            face = PointStamped()
            face.header = msg.header
            face.point = msg.pos
            self.listener.waitForTransform(face.header.frame_id, 'base_link', rospy.Time.now(), rospy.Duration(5.0))
            d = sqrt(face.point.x * face.point.x + face.point.y * face.point.y)
       
            # Change the axes from camera-type axes 
            self.quaternion = quaternion_from_euler(pi/2, pi/2, 0.0)
            pose = PoseStamped()
            pose.header = face.header
            pose.pose.position = face.point
            pose.pose.orientation.x = self.quaternion[0]
            pose.pose.orientation.y = self.quaternion[1]
            pose.pose.orientation.z = self.quaternion[2]
            pose.pose.orientation.w = self.quaternion[3]

            # Transform to base_link
            pose = self.listener.transformPose('base_link', pose)
            face = pose.pose.position
            self.quaternion = (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)


            self.origin = (face.x, face.y, face.z)

            # Flip the bit
            self.found_face = True

    def spin(self, rate=50):
        r = rospy.Rate(rate)

        while not rospy.is_shutdown():
            if self.found_face:
                self.broadcaster.sendTransform(self.origin,
                                          self.quaternion,
                                          rospy.Time.now(),
                                          '/face_detection',
                                          '/base_link')
            r.sleep()


if __name__ == '__main__':
    rospy.init_node('face_frame')

    head_frame = HeadFrame()
    head_frame.spin()

