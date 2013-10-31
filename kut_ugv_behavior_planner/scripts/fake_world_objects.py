#!/usr/bin/env python
import numpy as np
from math import *
import operator

import rospy
import roslib
import tf
import tf_conversions.posemath as tf_pm
import PyKDL as kdl
# Messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from kut_ugv_vehicle.msg import StateStamped as VehicleState
from kut_ugv_msgs.msg import WorldObject

roslib.load_manifest('kut_ugv_behavior_planner')

class RosFakeWorldObjects:
    """docstring for RosFakeWorldObjects"""

    world_object_pub = rospy.Publisher('/world_object', WorldObject)

    # objects_list = [{'x':  134.050, 'y':    4.250, 'type': WorldObject.MARK_STOPLINE},
    #                 {'x':  152.035, 'y':    1.483, 'type': WorldObject.LIGHT_GREEN},
    #                 {'x':  256.418, 'y':    9.516, 'type': WorldObject.LIGHT_ARROW_LEFT},
    #                 {'x':  492.842, 'y':   80.941, 'type': WorldObject.SIGN_SPEED_MAXIMUM},
    #                 {'x':  448.210, 'y':  164.566, 'type': WorldObject.SIGN_SPEED_MINIMUM},
    #                 {'x':  391.174, 'y':  205.739, 'type': WorldObject.SIGN_ROAD_WORKS},
    #                 {'x': -374.546, 'y': -113.727, 'type': WorldObject.SIGN_PEDESTRIAN_ZONE},
    #                 {'x': -424.904, 'y': -119.716, 'type': WorldObject.OBSTACLE_PEDESTRIAN},
    #                 {'x': -176.990, 'y': -116.173, 'type': WorldObject.SIGN_CROSSROAD},
    #                 {'x': -170.454, 'y':  -56.803, 'type': WorldObject.OBSTACLE_SPEEDBUMP},
    #                 {'x': -169.786, 'y':  -45.217, 'type': WorldObject.MARK_STOPLINE},
    # ]
    objects_list = [{'y': 3847103.145, 'x': 263301.865, 'type': WorldObject.MARK_STOPLINE},
                    {'y': 3847100.378, 'x': 263319.850, 'type': WorldObject.LIGHT_GREEN},
                    {'y': 3847108.411, 'x': 263424.233, 'type': WorldObject.LIGHT_ARROW_LEFT},
                    {'y': 3847179.836, 'x': 263660.657, 'type': WorldObject.SIGN_SPEED_MAXIMUM},
                    {'y': 3847263.461, 'x': 263616.025, 'type': WorldObject.SIGN_SPEED_MINIMUM},
                    {'y': 3847304.634, 'x': 263558.989, 'type': WorldObject.SIGN_ROAD_WORKS},
                    {'y': 3846985.168, 'x': 262793.269, 'type': WorldObject.SIGN_PEDESTRIAN_ZONE},
                    {'y': 3846979.179, 'x': 262742.911, 'type': WorldObject.OBSTACLE_PEDESTRIAN},
                    {'y': 3846982.722, 'x': 262990.825, 'type': WorldObject.SIGN_CROSSROAD},
                    {'y': 3847042.092, 'x': 262997.361, 'type': WorldObject.OBSTACLE_SPEEDBUMP},
                    {'y': 3847053.678, 'x': 262998.029, 'type': WorldObject.MARK_STOPLINE},
    ]

    detection_distance = 30.0

    def __init__(self):
        xy = rospy.get_param('/gps_ekf/origin', {'x': 0.0, 'y': 0.0})
        if rospy.get_param('/gps_ekf/relative_gps', True):
            rospy.loginfo('Using relative GPS. Origin at: %7.3f, %7.3f' % (xy['x'],xy['y']))
            self.objects_list = [{'x': obj['x'] - xy['x'], 'y': obj['y'] - xy['y'], 'type': obj['type']} for obj in self.objects_list]
            self.kdl_obj_list = [(kdl.Vector(obj['x'], obj['y'], 0.0), obj['type']) for obj in self.objects_list]

        # self.objects_list = [{'x': xy['x'] + obj['x'], 'y': xy['y'] + obj['y'], 'type': obj['type']} for obj in self.objects_list]
        # for obj in self.objects_list:
        #     print str(obj)+','

    def detect_objects(self, pose_msg):
        p0 = tf_pm.fromMsg(pose_msg)

        # keep those objects in the front half-plane which are closer than detection_distance
        objs = [obj for obj in self.kdl_obj_list if ((p0.Inverse() * obj[0]).x() > 0.0) and ((p0.Inverse() * obj[0]).Norm() < self.detection_distance)]
        # sort them
        objs = sorted(objs, key = lambda o: o[0].Norm())
        
        return objs

    def gps_odom_callback(self, msg):
        cur_x = msg.pose.pose.position.x
        cur_y = msg.pose.pose.position.y

        closest_obj = self.detect_objects(msg.pose.pose)

        for obj in closest_obj:
            self.publish_world_object(obj[0].x(), obj[0].y(), obj[1])

    def vehicle_state_callback(self, msg):
        pass

    def publish_world_object(self, x, y, obj_type):
        msg = WorldObject()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = '/odom'
        
        msg.type = obj_type
        msg.pose.position = Point(x, y, 0.0)
        msg.pose.orientation = Quaternion(*(kdl.Rotation.RPY(0.0, 0.0, 0.0).GetQuaternion()))
        
        rospy.logdebug('Publish object: %6.4f, %6.4f, %s' %(x, y, obj_type))
        self.world_object_pub.publish(msg)

    def publish(self, publish_rate):
        loop_rate = rospy.Rate(publish_rate)

        while not rospy.is_shutdown():
            try:
                loop_rate.sleep()
            except rospy.ROSException, e:
                if e.message == 'ROS time moved backwards':
                    rospy.logwarn("Saw a negative time change")

if __name__ == '__main__':
    rospy.init_node('fake_world_objects')

    fake_wo = RosFakeWorldObjects()

    gps_odom = rospy.get_param('~gps_odom_topic', '/ekf_odom')
    # vehicle_state = rospy.get_param('~vehicle_state_topic', '/vehicle/state')
    publish_rate = rospy.get_param('~publish_rate', 30)

    rospy.Subscriber(gps_odom, Odometry, fake_wo.gps_odom_callback, queue_size = 100)
    # rospy.Subscriber(vehicle_state, VehicleState, fake_wo.vehicle_state_callback, queue_size = 1)
    
    try:
        fake_wo.publish(publish_rate)
    except rospy.ROSInterruptException:
        rospy.logdebug("Exiting")
        pass
