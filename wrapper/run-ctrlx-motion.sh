#!/bin/bash
# Sources the shared ROS 2 runtime (mounted from the ros-base content snap),
# then this workspace's own install tree, then execs the node.
#
# Everything under $SNAP/rosruntime comes from the ROS runtime snap via the
# content interface; everything else under $SNAP is this workspace's colcon
# install/ directory.

export ROS_BASE=$SNAP/rosruntime
export PYTHONPATH=$PYTHONPATH:$ROS_BASE/lib/python3.12/site-packages
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ROS_BASE/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ROS_BASE/lib/$TRIPLET
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ROS_BASE/usr/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ROS_BASE/usr/include/comm/datalayer/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ROS_BASE/usr/lib/x86_64-linux-gnu/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ROS_BASE/usr/lib/$TRIPLET

export PATH=${PATH}:${ROS_BASE}/opt/ros/jazzy/bin
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${ROS_BASE}/opt/ros/jazzy/lib

# ROS 2 from the debian-package-based runtime snap.
source $ROS_BASE/opt/ros/jazzy/setup.bash

# This workspace: axis_status_msg, ctrlx_motion_action, motion_action_cpp.
# Needed so rclcpp can find the message/action typesupport libraries.
source $SNAP/local_setup.bash

source $SNAP/local_setup.bash && \
  $SNAP/motion_action_cpp/lib/motion_action_cpp/ctrlx_motion_node \
    --ros-args \
    --params-file $SNAP/motion_action_cpp/share/motion_action_cpp/config/ctrlx_motion.yaml
