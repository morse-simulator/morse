ROS installation for MORSE 
==========================

Blender 2.5x relies on Python3.x which is currently (Jan 2011) not supported by ROS.

The following steps explains how to get a working Python3 ROS setup, suitable for use with MORSE.

#. Install ROS unstable (check http://www.ros.org/wiki/unstable if needed)
#. Install Python3.x (these instructions where tested with Python3.1) manually or using
   your system package manager
#. Install PyYAML with Python3 support (PyYAML >= 3.09, you can get it from http://pyyaml.org/)

  - Install it with ``python3.1 setup.py install`` to be sure to have the Python3 libraries

#. Replace the ``ros`` and ``ros_comm`` stacks with the versions found here:

  - ``git clone http://code.in.tum.de/git/rospy3-stacks-ros.git``
  - ``git clone http://code.in.tum.de/git/rospy3-stacks-ros-comm.git``
  
#. Add your the path to your ``roslib-stack`` to your Pythonpath (this is needed for MORSE to load ROS manifest-files)

  - `` export PYTHONPATH=${YOUR_ROS_INSTALLATION_PATH}/ros/core/roslib/src:${PYTHONPATH}``

Check everything run fine by:

#. making sure your default Python executable is Python 3.x (``python --version``)
#. running ``rosmake --pre-clean ros`` and ``rosmake --pre-clean ros_comm``
