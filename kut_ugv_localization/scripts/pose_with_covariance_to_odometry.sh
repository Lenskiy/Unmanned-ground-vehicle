#!/usr/bin/env bash

unbuffer rostopic echo /odom_combined | unbuffer -p rosrun kut_ugv_localization recompose.py "{header: {stamp: 'header/stamp', frame_id: '\"odom_combined\"'}, child_frame_id: '\"base_footprint\"', pose: pose, twist: {covariance: 'pose/covariance', twist: {linear: {x: 'pose/covariance[5]', y: 'pose/covariance[5]', z: 'pose/covariance[5]'}, angular: {x: 'pose/covariance[5]', y: 'pose/covariance[5]', z: 'pose/covariance[5]'}}}}" | unbuffer -p tee dump.txt | rostopic pub -r 100 /ekf/odom nav_msgs/Odometry
