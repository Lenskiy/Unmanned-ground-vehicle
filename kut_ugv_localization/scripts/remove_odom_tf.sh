#!/bin/bash

if [ $# -lt 2 ]
then
  echo "Usage: remove_odom_tf input.bag output.bag\n"
  exit 1
fi

rosbag filter $1 $2 'topic != "/tf" or ((m.transforms[0].header.frame_id != "/odom" and m.transforms[0].header.frame_id != "odom") and (m.transforms[0].child_frame_id != "/base_footprint" and m.transforms[0].child_frame_id != "base_footprint"))'

