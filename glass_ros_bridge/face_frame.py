#!/usr/bin/env python

import roslib; roslib.load_manifest('face_frame')
import rospy

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import PointStamped

import tf
from tf.transformations import quaternion_from_euler

from math import sqrt, atan2


class HeadFrame:
    def __init__(self):
        self.found_face = False

        # Set up a tf broadcaster
        broadcaster = tf.TransformBroadcaster()

        # Set up the face detector callback
        self.sub = rospy.Subscriber('face_detector/faces_cloud',
                                    PointCloud,
                                    self.face_callback)

    def face_callback(self, msg):
        if len(msg.points) > 0 and not self.found_face:
            # Pick the closest detected face
            v,i = min(msg.points.z)

            # Transform the face position into the base_link frame
            face = PointStamped()
            face.header = msg.header
            face.point = msg.points[i]
            face = self.listener.transformPoint('base_link', face)

            # Set the origin of the face (in base_link)
            self.origin = (face.point.x, face.point.y, face.point.z)

            # Set the angles of the face frame.  Fake it for now, by
            # looking at (0, 0, 1.2) in base_link
            d = sqrt(face.point.x * face.point.x + face.point.y * face.point.y)
            self.quaternion = quaternion_from_euler(0.0, # roll
                                                    atan2(1.2, d), # pitch
                                                    atan2(face.point.y,  # yaw
                                                          face.point.z))

            # Flip the bit
            self.found_face = True

    def spin(self, rate=50):
        r = rospy.Rate(rate)

        while not rospy.is_shutdown():
            if self.found_face:
                broadcaster.sendTransform(self.origin,
                                          self.quaternion,
                                          rospy.Time.now(),
                                          'head',
                                          'base_link')
            r.sleep()


if __name__ == '__main__':
    rospy.init_node('face_frame')

    head_frame = HeadFrame()
    head_frame.spin()

