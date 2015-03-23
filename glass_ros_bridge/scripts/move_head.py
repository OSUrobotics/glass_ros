#!/usr/bin/env python

import rospy, actionlib
from visualization_msgs.msg import Marker
from control_msgs.msg import PointHeadAction, PointHeadGoal


if __name__ == '__main__':
    rospy.init_node('move_head')

    pub = rospy.Publisher('gaze_point', Marker)

    head_client = actionlib.SimpleActionClient('/head_traj_controller/point_head_action', PointHeadAction)
    head_client.wait_for_server()

    marker = Marker()
    marker.header.frame_id = 'glass'
    marker.header.stamp = rospy.Time()
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
        
    marker.pose.position.x = 1
    marker.pose.position.y = 0
    marker.pose.position.z = 0

    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        gaze_point = PointHeadGoal()
        gaze_point.target.header.frame_id = 'glass'
        gaze_point.pointing_frame = 'high_def_optical_frame'
        gaze_point.min_duration = rospy.Duration(0.01)
        gaze_point.max_velocity = 1.0
        gaze_point.target.header.stamp = rospy.Time.now()
        gaze_point.target.point.x = 1
        gaze_point.target.point.y = 0
        gaze_point.target.point.z = 0
        head_client.send_goal(gaze_point)

        pub.publish(marker)

        rate.sleep()
