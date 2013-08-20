#!/usr/bin/env python
import SocketServer, time
import struct
import sys
import rospy, tf, numpy
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Imu, Illuminance
from geometry_msgs.msg import PoseStamped

# See enum values at http://developer.android.com/reference/android/hardware/Sensor.html
TYPE_ACCELEROMETER = 1
TYPE_ORIENTATION   = 3
TYPE_LIGHT         = 5

class SocketHandler(SocketServer.BaseRequestHandler):
    child_frame_id = 'glass'

    def handle(self):
        print "Handle"
        self.br = tf.TransformBroadcaster()
        self.imu_pub = rospy.Publisher('/android/imu', Imu)
        self.pose_pub = rospy.Publisher('/android/pose', PoseStamped)
        self.light_pub = rospy.Publisher('/android/light', Illuminance)
        self.glass_base_frame = '/face_detection'
        print "Made broadcaster"
        self.data = self.request.recv(16)
        while not rospy.is_shutdown() and self.data:
            if self.data:
                try:
                    sensor = struct.unpack('>i', self.data[:4])[0]
                    if sensor == TYPE_ORIENTATION:
                        rpy = numpy.radians(struct.unpack('>3f', self.data[4:]))
                        quat = quaternion_from_euler(*rpy)
                        print 'Orientation', quat
                        stamp = rospy.Time.now()
                        pose_msg = PoseStamped()
                        pose_msg.header.frame_id = self.child_frame_id
                        pose_msg.header.stamp = stamp
                        pose_msg.pose.orientation.w = 1
                        self.pose_pub.publish(pose_msg)
                        self.br.sendTransform((0,0,0), quat, stamp, self.child_frame_id, self.glass_base_frame)
                    elif sensor == TYPE_ACCELEROMETER:
                        ax, ay, az = struct.unpack('>3f', self.data[4:])
                        print 'IMU', ax, ay, az
                        imu = Imu()
                        imu.header.frame_id = self.glass_base_frame
                        imu.header.stamp = rospy.Time.now()
                        imu.orientation.w = 1
                        imu.linear_acceleration.x = ax
                        imu.linear_acceleration.y = ay
                        imu.linear_acceleration.z = az

                        self.imu_pub.publish(imu)
                    elif sensor == TYPE_LIGHT:
                        l, _, _ = struct.unpack('>3f', self.data[4:])
                        print 'Light', l
                        ill = Illuminance()
                        ill.header.frame_id = self.child_frame_id
                        ill.header.stamp = rospy.Time.now()
                        ill.illuminance = l

                        light_pub.publish(ill)
                    else:
                        print 'Unknown sensor:', sensor

                except Exception, e:
                    if type(e) == struct.error:
                        print "Couldn't unpack ", self.data.__repr__(), 'with length', len(self.data)
            self.data = self.request.recv(16)
        print 'Disconnect'

if __name__ == "__main__":
    HOST, PORT = "192.168.43.202", 9999
    if len(sys.argv) > 1:
        HOST, PORT = sys.argv[1], int(sys.argv[2])
    print 'Listening on %s:%s' % (HOST, PORT)
    # Create the server, binding to localhost on port 9999
    server = SocketServer.TCPServer((HOST, PORT), SocketHandler)

    rospy.init_node('glass_sensor_bridge')
    print 'ROS Node started. Listening for messages.'

    server.serve_forever()
    server.close()
