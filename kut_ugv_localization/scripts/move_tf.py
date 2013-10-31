#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('kut_ugv_localization')

import sys
import os
import math
import rosbag
import numpy as np
import tf
import PyKDL as kdl

def edit_bag(cfg):
    with rosbag.Bag(cfg[2], 'w') as outbag:
        for topic, msg, t in rosbag.Bag(cfg[1]).read_messages():
            if topic == '/tf':
                if msg.transforms[0].header.frame_id == '/base_link':
#                    qx = msg.transforms[0].transform.rotation.x
#                    qy = msg.transforms[0].transform.rotation.y
#                    qz = msg.transforms[0].transform.rotation.z
#                    qw = msg.transforms[0].transform.rotation.w
#                    x = msg.transforms[0].transform.translation.x
#                    y = msg.transforms[0].transform.translation.y
#                    z = msg.transforms[0].transform.translation.z
#                    
#                    frame_old = kdl.Frame(kdl.Rotation.Quaternion(qx, qy, qz, qw), kdl.Vector(x, y, z))
#                    transf = kdl.Frame(kdl.Rotation(), kdl.Vector(float(cfg[3]), float(cfg[4]), float(cfg[5])))
                    
                    x = msg.transforms[0].transform.translation.x + float(cfg[3])
                    y = msg.transforms[0].transform.translation.y + float(cfg[4])
                    z = msg.transforms[0].transform.translation.z + float(cfg[5])
                    msg.transforms[0].transform.translation.x = x
                    msg.transforms[0].transform.translation.y = y
                    msg.transforms[0].transform.translation.z = z

            outbag.write(topic, msg, t)
        outbag.close()

if __name__ == '__main__':
    cfg = sys.argv

    if len(cfg) == 6:
        try:
            edit_bag(cfg)
        except KeyboardInterrupt:
            print ''
    else:
        print 'Usage: ' + cfg[0].split('/')[-1].split('.')[0] + ' input_bag output_bag x y z'

