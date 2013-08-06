#!/usr/bin/env python
import SocketServer, time
import struct
import sys
import rospy, tf, numpy
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Imu

# See enum values at http://developer.android.com/reference/android/hardware/Sensor.html
TYPE_ACCELEROMETER = 1
TYPE_ORIENTATION   = 3

class SocketHandler(SocketServer.BaseRequestHandler):
    child_frame_id = 'glass'

    def handle(self):
        print "Handle"
        self.br = tf.TransformBroadcaster()
        self.imu_pub = rospy.Publisher('imu', Imu)
        print "Made broadcaster"
        rate = rospy.Rate(50)
        self.data = self.request.recv(1024)
        while not rospy.is_shutdown() and self.data:
            if self.data:
                try:
                    sensor = struct.unpack('>i', self.data[:4])[0]
                    if sensor == TYPE_ORIENTATION:
                        rpy = numpy.radians(struct.unpack('>3f', self.data[4:]))
                        quat = quaternion_from_euler(*rpy)
                        print 'Orientation', quat
                        self.br.sendTransform((0,0,0), quat, rospy.Time.now(), self.child_frame_id, 'world')
                    elif sensor == TYPE_ACCELEROMETER:
                        ax, ay, az = struct.unpack('>3f', self.data[4:])
                        print 'IMU', ax, ay, az
                        imu = Imu()
                        imu.header.frame_id = self.child_frame_id
                        imu.header.stamp = rospy.Time.now()
                        imu.orientation.w = 1
                        imu.linear_acceleration.x = ax
                        imu.linear_acceleration.y = ay
                        imu.linear_acceleration.z = az

                        self.imu_pub.publish(imu)
                    else:
                        print 'Unknown sensor:', sensor

                except Exception, e:
                    if type(e) == struct.error:
                        # import pdb; pdb.set_trace()
                        print "Couldn't unpack ", self.data.__repr__(), 'with length', len(self.data)
            self.data = self.request.recv(1024)
            rate.sleep()
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
