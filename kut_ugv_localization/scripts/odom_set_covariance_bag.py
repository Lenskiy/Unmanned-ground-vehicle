#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('kut_ugv_localization')

import sys
import os
import math
import rosbag
import numpy as np

def edit_bag(cfg):
    last_cov = np.identity(6);
    # set prior covariance
    last_cov[0,0], last_cov[1,1], last_cov[5,5] = 1e-2, 1e-2, 1e-6

    i = 0;
    last_time = rospy.Time(0);
    with rosbag.Bag(cfg[2], 'w') as outbag:
        for topic, msg, t in rosbag.Bag(cfg[1]).read_messages():
            i = i + 1
            # This also replaces tf timestamps under the assumption 
            # that all transforms in the message share the same timestamp
            if topic == cfg[3]:
                if i == 1: last_time = msg.header.stamp

                dt = (msg.header.stamp - last_time).to_sec()

                tc = np.array(msg.pose.covariance).reshape(6,6)
                tc[0,0] = 1e-1 # x_dot
                tc[1,1] = 1e-1 # y_dot
                tc[2,2] = 1e+9 # z_dot
                tc[3,3] = 1e+9 # roll_dot
                tc[4,4] = 1e+9 # pitch_dot
                tc[5,5] = 1e-4 # yaw_dot
                msg.twist.covariance = tuple(tc.ravel().tolist())

                pc = np.array(msg.pose.covariance).reshape(6,6)
                pc[0,0] = last_cov[0,0] + tc[0,0] * dt**2 * msg.twist.twist.linear.x**2 # x 
                pc[1,1] = last_cov[1,1] + tc[1,1] * dt**2 * msg.twist.twist.linear.y**2 # y 
                pc[2,2] = tc[2,2] # z 
                pc[3,3] = tc[3,3] # roll
                pc[4,4] = tc[4,4] # pitch
                pc[5,5] = last_cov[5,5] + tc[5,5] * dt**2 * msg.twist.twist.angular.z**2 # yaw
                last_cov = pc
                msg.pose.covariance = tuple(pc.ravel().tolist())
                
                last_time = msg.header.stamp

                sys.stdout.write('\rMessages processed: %d' %i)
                sys.stdout.flush()

            outbag.write(topic, msg, t)
        outbag.close()
        sys.stdout.write('\n')
        sys.stdout.flush()

if __name__ == '__main__':
    cfg = sys.argv

    if len(cfg) == 4:
        try:
            edit_bag(cfg)
        except KeyboardInterrupt:
            print ''
    else:
        print 'Usage: ' + cfg[0].split('/')[-1].split('.')[0] + ' <input_bag> <output_bag> <topic_name>'

