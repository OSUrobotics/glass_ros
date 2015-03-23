# self.Kalman filter example demo in self.Python

# A self.Python implementation of the example given in pages 11-15 of "An
# Introduction to the self.Kalman Filter" by Greg Welch and Gary Bishop,
# University of North Carolina at Chapel Hill, Department of Computer
# Science, TR 95-041,
# http://www.cs.unc.edu/~welch/kalman/kalmanIntro.html

# by Andrew D. Straw

import numpy as np
import pylab
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler

from highpass import HighPassFilter

# from testdata import data as z
# z = z[:300,:]

class KalmanFilter(object):

    def __init__(self, dim):
        # intial parameters
        # self.sz = z.shape # size of array
        self.sz = (1,dim)

        self.tmat =  np.matrix([ 
        #   [ax, ay, az, vr, vp, vh, vx, vy, vz, x,  y,  z,  r,  p,  h]
            [1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0], # = ax
            [0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0], # = ay
            [0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0], # = az
            [0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0], # = vr
            [0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0], # = vp
            [0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0], # = vh
            [2,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0], # = vx
            [0,  2,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0], # = vy
            [0,  0,  2,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0], # = vz
            [0,  0,  0,  0,  0,  0,  2,  0,  0,  1,  0,  0,  0,  0,  0], # = x
            [0,  0,  0,  0,  0,  0,  0,  2,  0,  0,  1,  0,  0,  0,  0], # = y
            [0,  0,  0,  0,  0,  0,  0,  0,  2,  0,  0,  1,  0,  0,  0], # = z
            [0,  0,  0,  2,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0], # = r
            [0,  0,  0,  0,  2,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0], # = p
            [0,  0,  0,  0,  0,  2,  0,  0,  0,  0,  0,  0,  0,  0,  1], # = h
        ])


        # allocate space for arrays
        self.xhat = np.zeros(self.sz)      # a posteri estimate of x

        # self.P = np.zeros(self.sz)         # a posteri error estimate
        self.P = np.asmatrix(np.zeros((dim,dim)))

        self.xhatminus = np.zeros(self.sz) # a priori estimate of x

        # self.Pminus = np.zeros(self.sz)    # a priori error estimate
        self.Pminus = np.asmatrix(np.zeros((dim,dim)))

        self.K = np.zeros(self.sz)         # gain or blending factor

        self.state = np.zeros(15)

        # self.R = 0.1**2 # estimate of measurement variance, change to see effect
        self.R = np.asmatrix(np.eye(dim)) * 0.1**1
        # self.Q = 1e-5 # process variance
        self.Q = np.asmatrix(np.eye(dim)) * 1e-5


        # intial guesses
        # self.xhat[0] = 0.0
        # self.P[0] = 1.0

    def update(self, z, dt=1.0):
        # import pdb; pdb.set_trace()

        # time update
        self.xhatminus = self.xhat
        self.Pminus = self.P+self.Q

        # measurement update
        self.K = np.nan_to_num(self.Pminus/(self.Pminus+self.R))

        # self.xhat = self.xhatminus + self.K*(z-self.xhatminus)
        self.xhat = self.xhatminus + (z.T - self.xhatminus)*self.K

        self.P = (np.eye(self.sz[1])-self.K) * self.Pminus

        # update the velocities and positions based on the filtered IMU data
        tmatdt = np.matrix(self.tmat, dtype=np.float64)
        tmatdt[np.nonzero(tmatdt == 2)] = dt

        # import pdb; pdb.set_trace()
        self.state[:self.xhat.shape[1]] = self.xhat
        self.state = np.asarray(tmatdt * np.asmatrix(self.state).T).squeeze()

        return self.xhat, self.state


filt = None
last_time = None
odom_pub = None
odom_pose_pub = None
imu_pub = None

# bias = np.matrix([0, -0.014, 0.012, 0.0003, 0.0003, 0.0002]).T

highpass = HighPassFilter(0.8)

def imu_cb(msg):
    time = msg.header.stamp
    global last_time
    # print last_time
    if last_time:
        ang = msg.angular_velocity
        lin = msg.linear_acceleration
        signal = np.matrix([
            0.0,
            lin.y, # if abs(lin.y) > 0.025 else 0,
            lin.z, # if abs(lin.z) > 0.025 else 0,
            ang.x,
            ang.y,
            ang.z]
        ).T

        # signal = highpass.filter(signal)


        # print signal
        # signal = np.matrix([lin.x-9.973, lin.y, lin.z, ang.x, ang.y, ang.z]).T
        estimate, state = np.asarray(filt.update(signal, (time-last_time).to_sec())).flatten()
        # print state[9] + '\t', state[10] + '\t' + state[11]
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = '/face_detection'
        # odom.child_frame_id = '/glass'
        odom.child_frame_id = '/face_detection'
        pose = odom.pose.pose
        ps = PoseStamped()
        ps.header = odom.header
        ps.pose = pose
        twist = odom.twist.twist

        pose.position.x, pose.position.y, pose.position.z = state[9:12]
        # pose.position.x = 0

        # print '%0.4f, %0.4f, %0.4f' % tuple(state[9:12])
        # print '\t'.join('%0.4f' % n for n in signal[:3])
        quat = quaternion_from_euler(*state[12:15])
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat

        # twist.angular.x, twist.angular.y, twist.angular.z = state[12:15]
        twist.angular.x, twist.angular.y, twist.angular.z = state[3:6]

        # print '%s, %s, %s, %s, %s, %s' % tuple(np.asarray(estimate[0,:6]).squeeze().tolist())
        print '%s, %s, %s, %s, %s, %s' % tuple(np.asarray(signal).squeeze().tolist())

        odom_pub.publish(odom)
        odom_pose_pub.publish(ps)

    last_time = time


if __name__ == '__main__':
    filt = KalmanFilter(6)

    rospy.init_node('imu_kalman')
    odom_pub = rospy.Publisher('/android/odom', Odometry)
    odom_pose_pub = rospy.Publisher('/android/odom_pose', PoseStamped)
    imu_pub = rospy.Publisher('/android/imu_filtered', Imu)

    rospy.Subscriber('/android/imu', Imu, imu_cb)
    rospy.spin()


    # for k in range(1,n_iter):
    #     estimate, state = filt.update(z[k])

    # pylab.figure()
    # pylab.plot(z,'k+',label='noisy measurements')
    # pylab.plot(xhat,'b-',label='a posteri estimate')
    # pylab.legend()
    # pylab.xlabel('Time')
    # pylab.ylabel('Acceleration')
    # pylab.show()
