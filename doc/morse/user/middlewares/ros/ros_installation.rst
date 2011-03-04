ROS installation for MORSE 
==========================

Blender 2.5x relies on Python3.x which is currently (Jan 2011) not supported by ROS.

The following steps explains how to get a working Python3 ROS setup, suitable for use with MORSE.

#. Install ROS unstable (check http://www.ros.org/wiki/unstable if needed)
#. Install Python3.x (these instructions where tested with Python3.1) manually or using
   your system package manager
#. Install PyYAML with Python3 support (PyYAML >= 3.09, you can get it from http://pyyaml.org/)

  - Install it with ``python3.1 setup.py install`` to be sure to have the Python3 libraries

#. Create an overlay of your ROS installation using rosinstall: 
   
   ``rosinstall <path-to-your-overlay> <your-ROS-installation-path>``

   Download the Python3-compatible ros and ros_comm-stacks here:   

  - ``git clone http://code.in.tum.de/git/rospy3-stacks-ros.git``
  - ``git clone http://code.in.tum.de/git/rospy3-stacks-ros-comm.git``

    Rename the ``rospy-stacks-ros`` folder to ``ros``
    Rename the ``rospy-stack-ros_comm`` folder to ``ros_comm``

    Install the Python3-compatible stacks to your Overlay using rosinstall: 

  - ``rosinstall <path-to-your-overlay> <path-to-your-overlay>/ros``
  - ``rosinstall <path-to-your-overlay> <path-to-your-overlay>/ros_comm`` 

    Change your ROS_ROOT in the setup.sh in your ROS overlay to the folder of the overlayed ros-stack. So the first line of your setup.sh should look like this:

   ``export ROS_ROOT=<path-to-your-ROS-overlay>/ros``  

#. Add your the path to your ``roslib-stack`` to your Pythonpath (this is needed for MORSE to load ROS manifest-files)

  - ``export PYTHONPATH=${YOUR_ROS_INSTALLATION_PATH}/ros/core/roslib/src:${PYTHONPATH}``

Check everything run fine by:

#. running ``rosmake --pre-clean ros`` and ``rosmake --pre-clean ros_comm``   

You will also have to remake the messages you want to use to be Python3-compatible by using rosmake --pre-clean. For example for std_msgs, rosgraph_msgs and sensor_msgs type the following:

``rosmake --pre-clean std_msgs rosgraph_msgs sensor_msgs``  

