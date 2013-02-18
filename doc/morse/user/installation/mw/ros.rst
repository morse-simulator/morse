ROS Installation Notes
~~~~~~~~~~~~~~~~~~~~~~

ROS Groovy
----------

Same as `ROS Fuerte`_ with just few more steps.

Run ``sudo rosdep init`` and ``rosdep update``

Install catkin for Python 3 support::

    git clone https://github.com/ros-infrastructure/catkin_pkg
    cd catkin_pkg/
    python3 setup.py install
    sudo python3 setup.py install

    git clone https://github.com/ros/catkin
    cd catkin/
    python3 setup.py install
    sudo python3 setup.py install


ROS Fuerte
----------

MORSE is based on Blender and requires Python 3.2. Python 3 is
partially supported by **ROS Fuerte** from patch release 1.8.15, 
which was rolled out on 20/07/2012. Any ROS Fuerte installation 
from that day on works fine with MORSE.

First, install MORSE using the 
:doc:`installation instructions  <../../installation>`.
Make sure to set the **BUILD_ROS_SUPPORT** parameter to **ON** when
calling cmake! *eg.*::

    cmake -DBUILD_ROS_SUPPORT=ON ..

#. Install ROS Fuerte if needed: http://ros.org/wiki/fuerte#Installation

#. Install Python 3.2 using your system package manager (available in Ubuntu >=
   11.04) or manually from the sources, and make sure your ``$PYTHONPATH``
   variable includes the Python3.2 libraries::

        sudo apt-get install python3.2-dev

#. Install ``PyYAML`` with Python3 support (package ``python3-yaml`` on
   Debian/Ubuntu, or you can get the sources from http://pyyaml.org/ ) and
   build it using python3.2::

        wget http://pyyaml.org/download/pyyaml/PyYAML-3.10.tar.gz
        tar xvf PyYAML-3.10.tar.gz
        cd PyYAML-3.10
        sudo python3.2 setup.py


#. Install rospkg using Python3.2::

        curl -O http://python-distribute.org/distribute_setup.py
        python3 distribute_setup.py install

        git clone git://github.com/ros/rospkg.git
        cd rospkg
        sudo python3.2 setup.py install

#. Open a terminal and check if everything is correctly set. Therefore, open
   a terminal and type:

   ``morse check``

   If successful, the following line will be printed after some other information 
   about your configuration:

   ``* Your environment is correctly setup to run MORSE.``

#. Done. You can start having fun with MORSE and ROS!


ROS Electric and Diamondback
----------------------------

To get a working setup suitable for using **ROS Electric** or **Diamondback** with 
MORSE, please follow the following steps:

#. Install ROS Electric Emys (check http://www.ros.org/wiki/ROS/Installation if
   needed)

#. Install Python 3.2 using your system package manager (available in Ubuntu >=
   11.04) or manually from the sources, and make sure your ``$PYTHONPATH``
   variable includes the Python3.2 libraries.

#. Install ``PyYAML`` with Python3 support (package ``python3-yaml`` on
   Debian/Ubuntu, or you can get the sources from http://pyyaml.org/. Install
   the source by running ``python3.2 setup.py install`` to be sure to have the
   Python 3 libraries.

#. Due to the lacking Python 3 compatibility, you have to overlay some ROS
   stacks. You can use ``rosinstall`` to this end:

   ROS Electric::

        rosinstall ~/ros-py3  http://ias.cs.tum.edu/~kargm/ros_electric_py3.rosinstall

   (Assuming your ROS is installed in ``/opt/ros/electric`` and your overlay should 
   be created in ``~/ros-py3``)
       
   **Note:** rosinstall will create a new setup.bash in the folder *~/ros-py3/*.
   It has to be sourced (instead of the original setup.bash) before using MORSE 
   with ROS.

   You can also use MORSE with ROS Diamondback::

        rosinstall ~/ros-py3 http://ias.cs.tum.edu/~kargm/ros_diamondback_py3.rosinstall

   (assuming your ROS is installed in ``/opt/ros/diamondback`` and your overlay should 
   be created in ``~/ros-py3``) 

   The ROS-stacks ros, ros_comm and common_msgs are overlayed by Python3-compatible
   versions and need to be rebuild::

        rosmake ros && rosmake ros_comm && rosmake common_msgs

   **Note:** Rebuilding the common_msgs stack allows you to use all messages in this
   stack for communicating between MORSE and ROS. If you want to use any other
   messages, make sure the source-files are Python2 AND Python3 compatible! This
   can be achieved by simply rebuilding the ROS-packages of the messages with
   rosmake --pre-clean when you are running the patched ROS-stacks (make sure to
   source the right setup.bash!), e.g.::

        rosmake --pre-clean sensor_msgs

Troubleshooting
---------------

- ``urandom error`` : make sure your ``python3`` version is equal to the one in
  Blender.
- ``No module named rospkg`` : install rospkg with Python3 as in `ROS Fuerte`_ #4

Resources
---------

- http://ros.org/wiki/diamondback
- http://ros.org/wiki/electric
- http://ros.org/wiki/fuerte
- http://ros.org/wiki/groovy
