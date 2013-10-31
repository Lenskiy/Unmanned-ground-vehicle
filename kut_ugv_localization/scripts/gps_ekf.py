#!/usr/bin/env python
import numpy as np
from math import *

import rospy
import roslib
import tf
import PyKDL as kdl
# Messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from kut_ugv_vehicle.msg import StateStamped as VehicleState

roslib.load_manifest('kut_ugv_localization')

class RosGpsEkf:
    """docstring for RosGpsEkf"""

    gps_ekf_odom_pub = rospy.Publisher('/ekf_odom', Odometry)
    tf_br = tf.TransformBroadcaster()

    relative_gps = False
    publish_odom_tf = True

    frame_id = ''
    child_frame_id = ''

    gps_rate = 20;

    def __init__(self):
        self.ekf_reset()

    def ekf_reset(self):
        # EKF variables
        self.Q = np.mat(np.diag([1e-1, 1e-1, 1e0]))
        self.P = np.mat(np.diag([0.0]*3))
        # 2 x 2 matrix
        self.R_meas = np.mat(np.diag([0.0]*2))
        self.F = np.mat(np.diag([1.0]*3))
        self.H = np.mat('1 0 0; 0 1 0')
        self.Z = np.mat('0.0; 0.0')
        self.X_post = np.mat('0.0; 0.0; 0.0')
        self.X_pred = np.mat('0.0; 0.0; 0.0')
        self.X_filtered = [[0.0, 0.0, 0.0]]
        self.ori_x = [[.0,.0]]
        self.ori_y = [[.0,.0]]
        # initialize variables
        self.x_meas, self.y_meas, self.x_cov, self.y_cov, self.vel, self.alpha = [0.0]*6
        self.last_time_vehicle = rospy.Time(0)
        self.last_time_gps = rospy.Time(0)
        self.filter_initialized = False
        self.old_gps_x = 0.0
        self.old_gps_y = 0.0
 
        self.origin_gps_x = 0.0
        self.origin_gps_y = 0.0
    
    def bicycle_model(self, x, y, theta, vel, alpha, dt):
        L = 2.7
        d = vel * dt
        beta = d/L * tan(alpha)
        
        if abs(beta) < 0.001:
            x = x + d * cos(theta)
            y = y + d * sin(theta)
            theta = theta + beta;
            x_dot = -d * sin(theta)
            y_dot = d * cos(theta)
        else:
            R = d / beta
            Cx = x - sin(theta) * R
            Cy = y + cos(theta) * R
            theta = theta + beta
            x = Cx + sin(theta) * R
            y = Cy - cos(theta) * R
            x_dot = -R * cos(theta - beta) + R * cos(theta)
            y_dot = -R * sin(theta - beta) + R * sin(theta)
            
        return x, y, theta, x_dot, y_dot

    def ekf_init(self):
        self.X_post = np.mat([self.x_meas, self.y_meas, 0.0]).T
        self.P[0,0] = self.x_cov
        self.P[1,1] = self.y_cov
        self.P[2,2] = 1e9
        self.R_meas[0,0] = self.x_cov
        self.R_meas[1,1] = self.y_cov

    def ekf_update(self, dt):
        new_gps_x = self.x_meas
        new_gps_y = self.y_meas

        # prediction
        x, y, theta = self.X_post[0,0], self.X_post[1,0], self.X_post[2,0]
        
        #self.X_filtered.append([x, y, theta])
        
        x, y, theta, x_dot, y_dot = self.bicycle_model(x, y, theta, self.vel, self.alpha, dt)

        if new_gps_x == self.old_gps_x and new_gps_y == self.old_gps_y:
            #rospy.logwarn("Wrong (duplicated) GPS data. Extrapolating...")
            self.X_post[0,0] = x
            self.X_post[1,0] = y
            self.X_post[2,0] = theta
            return

        self.old_gps_x = new_gps_x
        self.old_gps_y = new_gps_y

        self.X_pred[0,0] = x
        self.X_pred[1,0] = y
        self.X_pred[2,0] = theta

        #self.ori_x.append([x, cos(theta)])
        #self.ori_y.append([y, sin(theta)])
        
        # model linearization
        self.F[0,2] = x_dot
        self.F[1,2] = y_dot
        
        self.P = self.F * self.P * self.F.T + self.Q
        
        self.R_meas[0,0] = self.x_cov
        self.R_meas[1,1] = self.y_cov
        
        self.Z[0,0] = self.x_meas
        self.Z[1,0] = self.y_meas
        
        self.Y = self.Z - self.X_pred[0:2]
        self.S = self.H * self.P * self.H.T + self.R_meas
        self.K = self.P * self.H.T * self.S.I
        
        self.X_post = self.X_pred + self.K * self.Y
        
        self.P = (np.eye(3) - self.K * self.H) * self.P

    def gps_odom_callback(self, msg):
        dt = (msg.header.stamp - self.last_time_gps).to_sec()

        cov = np.array(msg.pose.covariance).reshape(6,6)
        new_gps_x = msg.pose.pose.position.x
        new_gps_y = msg.pose.pose.position.y

        if not self.filter_initialized:
            self.ekf_reset()

            self.last_time_vehicle = self.last_time_gps = msg.header.stamp
            self.ekf_init()
            #self.child_frame_id = msg.header.frame_id

            if self.relative_gps:
                self.origin_gps_x = new_gps_x
                self.origin_gps_y = new_gps_y
                rospy.set_param('~origin/x', new_gps_x)
                rospy.set_param('~origin/y', new_gps_y)
                rospy.loginfo("Using relative GPS coordinates. Origin stored in ~/origin")

            self.old_gps_x = new_gps_x - self.origin_gps_x
            self.old_gps_y = new_gps_y - self.origin_gps_y

            self.filter_initialized = True
            rospy.loginfo("EKF initialized")
            return

        self.x_meas = new_gps_x - self.origin_gps_x
        self.y_meas = new_gps_y - self.origin_gps_y

        self.x_cov = cov[0,0]
        self.y_cov = cov[1,1]

        self.last_time_gps = msg.header.stamp
        
        self.ekf_update(dt)

    def vehicle_state_callback(self, msg):
        self.vel = msg.state.velocity
        self.alpha = msg.state.theta - 0.0095
        
        gps_dt = (msg.header.stamp - self.last_time_gps).to_sec()

        # if  gps_dt > 0.05: #((1/self.gps_rate) * 10):
        #     if not self.filter_initialized: return
        #     rospy.logwarn('GPS got lost, last update was ' + str(gps_dt) + 's ago')
        #     dt = (msg.header.stamp - self.last_time_vehicle).to_sec()
        #     self.last_time_vehicle = rospy.Time(0.05 + msg.header.stamp.to_sec())
        #     self.last_time_gps = rospy.Time(0.05 + msg.header.stamp.to_sec())
        #     # update with fake values
        #     x, y, theta = self.X_post[0,0], self.X_post[1,0], self.X_post[2,0]
        #     x, y, theta, x_dot, y_dot = self.bicycle_model(x, y, theta, self.vel, self.alpha, 0.05)
        #     self.X_post[0,0], self.X_post[1,0], self.X_post[2,0] = x, y, theta
        #     self.x_meas, self.y_meas = x, y
        #     #self.x_cov, self.y_cov = 1e-1, 1e-1
        #     #self.P[2,2] = 10e9
        #     #self.ekf_update(0.01)
        #     self.publish_odom()

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.last_time_vehicle if self.last_time_vehicle > self.last_time_gps else self.last_time_gps
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id
        
        msg.pose.pose.position = Point(self.X_post[0,0], self.X_post[1,0], 0.0)
        msg.pose.pose.orientation = Quaternion(*(kdl.Rotation.RPY(0.0, 0.0, self.X_post[2,0]).GetQuaternion()))
        
        p_cov = np.array([0.0]*36).reshape(6,6)
        
        # position covariance
        p_cov[0:2,0:2] = self.P[0:2,0:2]
        # orientation covariance for Yaw
        # x and Yaw
        p_cov[5,0] = p_cov[0,5] = self.P[2,0]
        # y and Yaw
        p_cov[5,1] = p_cov[1,5] = self.P[2,1]
        # Yaw and Yaw
        p_cov[5,5] = self.P[2,2]
        
        msg.pose.covariance = tuple(p_cov.ravel().tolist())

        pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        ori = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        self.gps_ekf_odom_pub.publish(msg)

        if self.publish_odom_tf:
            self.tf_br.sendTransform(pos, ori, msg.header.stamp, msg.child_frame_id, msg.header.frame_id)

    def publish(self, publish_rate):
        loop_rate = rospy.Rate(publish_rate)

        while not rospy.is_shutdown():
            if not self.filter_initialized:
                loop_rate.sleep()
                continue

            self.publish_odom()
            try:
                loop_rate.sleep()
            except rospy.ROSException, e:
                if e.message == 'ROS time moved backwards':
                    rospy.logwarn("Saw a negative time change, resetting GPS EKF.")
                    self.filter_initialized = False

if __name__ == '__main__':
    rospy.init_node('gps_ekf_node')

    gps_ekf = RosGpsEkf()

    gps_odom = rospy.get_param('~gps_odom_topic', '/gps_xy')
    vehicle_state = rospy.get_param('~vehicle_state_topic', '/vehicle/state')
    publish_rate = rospy.get_param('~publish_rate', 30)
    gps_ekf.gps_rate = rospy.get_param('~gps_rate', 20)

    gps_ekf.frame_id = rospy.get_param('~frame_id', '/odom')
    gps_ekf.child_frame_id = rospy.get_param('~child_frame_id', '/base_footprint')

    # Set GPS position relative to first measurement, i.e. first measurement will be at (0,0)
    gps_ekf.relative_gps = rospy.get_param('~relative_gps', True)

    gps_ekf.publish_odom_tf = rospy.get_param('~publish_odom_tf', True)
    
    rospy.Subscriber(gps_odom, Odometry, gps_ekf.gps_odom_callback, queue_size = 1)
    rospy.Subscriber(vehicle_state, VehicleState, gps_ekf.vehicle_state_callback, queue_size = 1)
    
    try:
        gps_ekf.publish(publish_rate)
    except rospy.ROSInterruptException:
        rospy.logdebug("Exiting")
        pass
