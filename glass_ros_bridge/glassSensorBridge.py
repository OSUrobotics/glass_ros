#!/usr/bin/env python
import SocketServer, time
import struct
import sys
import rospy, tf, numpy
from tf.transformations import quaternion_from_euler

class SocketHandler(SocketServer.BaseRequestHandler):
    def handle(self):
        print "Handle"
        self.br = tf.TransformBroadcaster()
        print "Made broadcaster"
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.data = self.request.recv(1024)
            if self.data:
                try:
                    rpy = numpy.radians(struct.unpack('>3f', self.data))
                    quat = quaternion_from_euler(*rpy)
                    print quat
                    self.br.sendTransform((0,0,0), quat, rospy.Time.now(), 'glass', 'world')

                except Exception, e:
                    if type(e) == struct.error:
                        print "Couldn't unpack ", self.data
        rate.sleep()

if __name__ == "__main__":
    HOST, PORT = "192.168.0.157", 9992
    if len(sys.argv) > 1:
        HOST, PORT = sys.argv[1], int(sys.argv[2])
    print 'Listening on %s:%s' % (HOST, PORT)
    # Create the server, binding to localhost on port 9999
    server = SocketServer.TCPServer((HOST, PORT), SocketHandler)

    rospy.init_node('glass_sensor_bridge')
    print 'ROS Node started. Listening for messages.'

    server.serve_forever()
    server.close()
