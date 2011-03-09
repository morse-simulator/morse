ROS installation for MORSE 
==========================

Blender 2.5x relies on Python3.x which is currently (Jan 2011) not supported by ROS.

The following steps explains how to get a working Python3 ROS setup, suitable for use with MORSE.

1. Install ROS Diamondback (check http://www.ros.org/wiki/ROS/Installation if needed)
2. Install Python3.x (these instructions were tested with Python3.1) manually or using
   your system package manager and make sure, your Pythonpath variable is pointing to the Python3-libraries
3. Install PyYAML with Python3 support (PyYAML >= 3.09, you can get it from http://pyyaml.org/)

  - Install it with ``python3.1 setup.py install`` to be sure to have the Python3 libraries

4. Create a Python3-compatible overlay of your ROS installation using rosinstall with our Python3-rosinstall-file: 
   
   - ``rosinstall <path-to-your-overlay> http://ias.cs.tum.edu/~kargm/ros_py3.rosinstall``

   The ROS-stacks ros, ros_comm and common_msgs are overlayed by Python3-compatible versions and need to be rebuild:

   - ``rosmake ros && rosmake ros_comm && rosmake common_msgs``

   Note: Rebuilding the common_msgs stack allows you to use all messages in this stack for communicating between MORSE and ROS. If you want to use any other messages, make sure the source-files are Python2 AND Python3 compatible!

5. Add the path to your ``roslib-stack`` to your Pythonpath (this is needed for MORSE to load ROS manifest-files)

  - ``export PYTHONPATH=${YOUR_ROS_OVERLAY_INSTALLATION_PATH}/ros/core/roslib/src:${PYTHONPATH}``



