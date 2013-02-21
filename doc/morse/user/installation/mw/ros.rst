ROS Installation Notes
~~~~~~~~~~~~~~~~~~~~~~

ROS Groovy
----------

Same as `ROS Fuerte`_ with just few more steps.

Run ``sudo rosdep init`` and ``rosdep update`` as mentionned in the
`installation wiki <http://ros.org/wiki/groovy/Installation/Ubuntu#Initialize_rosdep>`_

Install catkin for Python 3 support::

    git clone git://github.com/ros-infrastructure/catkin_pkg.git -b 0.1.9
    cd catkin_pkg
    sudo python3 setup.py install

    git clone git://github.com/ros/catkin.git
    cd catkin
    sudo python3 setup.py install


ROS Fuerte
----------

MORSE is based on Blender and requires Python 3. Python 3 is
partially supported by **ROS Fuerte**.

First, install MORSE using the 
:doc:`installation instructions  <../../installation>`.
Make sure to set the **BUILD_ROS_SUPPORT** parameter to **ON** when
calling cmake! *eg.*::

    cmake -DBUILD_ROS_SUPPORT=ON ..

#. Install ROS Fuerte if needed: http://ros.org/wiki/fuerte#Installation

#. Install Python 3 using your system package manager (available in Ubuntu >=
   11.04) or manually from the sources, and make sure your ``$PYTHONPATH``
   variable includes the Python 3 libraries::

        sudo apt-get install python3-dev

#. Install ``PyYAML`` with Python3 support (package ``python3-yaml`` on
   Debian/Ubuntu, or you can get the sources from http://pyyaml.org ) and
   build it using python3::

        wget http://pyyaml.org/download/pyyaml/PyYAML-3.10.tar.gz
        tar xvf PyYAML-3.10.tar.gz
        cd PyYAML-3.10
        sudo python3 setup.py

#. Install rospkg using Python3::

        wget http://python-distribute.org/distribute_setup.py
        sudo python3 distribute_setup.py

        git clone git://github.com/ros/rospkg.git
        cd rospkg
        sudo python3 setup.py install

#. Open a terminal and check if everything is correctly set. Therefore, open
   a terminal and type:

   ``morse check``

   If successful, the following line will be printed after some other information 
   about your configuration:

   ``* Your environment is correctly setup to run MORSE.``

#. Done. You can start having fun with MORSE and ROS!


Troubleshooting
---------------

- ``urandom error`` : make sure your ``python3`` version is equal to the one in
  Blender.
- ``No module named rospkg`` : install rospkg with Python3 as in `ROS Fuerte`_ #4

Resources
---------

- http://ros.org/wiki/fuerte
- http://ros.org/wiki/groovy
