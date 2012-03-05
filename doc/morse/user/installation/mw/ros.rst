ROS Installation Notes
~~~~~~~~~~~~~~~~~~~~~~

Blender 2.57+ relies on Python3.2 which is not yet supported by ROS Electric
Emys. 

The following steps explain how to get a working setup, suitable for using ROS
with MORSE.

#. Install ROS Electric Emys (check http://www.ros.org/wiki/ROS/Installation if
   needed)

#. Install Python3.2 manually or using your system package manager and make
   sure, your Pythonpath variable is pointing to the Python3.2-libraries
   (Python3.2 Debian-packages are e.g. offered by Ubuntu 11.04 and newer) 

#. Install PyYAML with Python3 support (PyYAML >= 3.09, you can get it from
   http://pyyaml.org/) Install it with ``python3.2 setup.py install`` to be sure
   to have the Python3 libraries

#. Due to the lacking Python 3 compatibility, you have to overlay some ROS
   stacks: Therefore, you can use rosinstall:

   ROS Electric:

   ``rosinstall ~/ros-py3 /opt/ros/electric
   http://ias.cs.tum.edu/~kargm/ros_electric_py3.rosinstall`` (if your ROS is
   installed in ``/opt/ros/electric`` and your overlay should be created in
   ``~/ros-py3``)
       
   **Note:** rosinstall will create a new setup.bash in the folder *~/ros-py3/*.
   It has to be sourced (instead of the original setup.bash) before using MORSE 
   with ROS.

   You can also use MORSE with ROS Diamondback:

   ``rosinstall ~/ros-py3 /opt/ros/diamondback
   http://ias.cs.tum.edu/~kargm/ros_diamondback_py3.rosinstall`` (if your ROS
   is installed in ``/opt/ros/diamondback`` and your overlay should be created
   in ``~/ros-py3``) 

   The ROS-stacks ros, ros_comm and common_msgs are overlayed by Python3-compatible
   versions and need to be rebuild: ``rosmake ros && rosmake ros_comm && rosmake
   common_msgs``

   **Note:** Rebuilding the common_msgs stack allows you to use all messages in this
   stack for communicating between MORSE and ROS. If you want to use any other
   messages, make sure the source-files are Python2 AND Python3 compatible! This
   can be achieved by simply rebuilding the ROS-packages of the messages with
   rosmake --pre-clean when you are running the patched ROS-stacks (make sure to
   source the right setup.bash!), e.g.: ``rosmake --pre-clean sensor_msgs``
