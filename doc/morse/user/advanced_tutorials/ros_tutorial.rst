ROS-navigation tutorial
=======================

This tutorial shows how to use the ROS navigation stack to navigate a robot in
MORSE.

Prerequisites
-------------

You should be familiar with the basic usage of ROS and how to use TF and
the ROS navigation stack. You should also know about launchfiles and topic
remapping as well as the robot state publisher. Also experience with RVIZ are
of advantage. Tutorials on all of those topics can be found on
http://www.ros.org/wiki/ROS/Tutorials.

We also assume you know how to use the MORSE Builder API to equip your robot
with components.  If not, please follow first the :doc:`Builder API and ROS
middleware  <./tutorial_builder_ros>` tutorial.

Tutorial setup
--------------

You need to have a working installation of ROS Electric or Diamonback and also
have the python3-compatible stacks for MORSE-ROS installed. You can find
information about this here:  :doc:`installation notes <../installation/mw/ros>`

.. note::
    We base the tutorial on ROS Electric. The tutorial is however also compatible with
    ROS Diamondback. When significant, differences are noted.

If you are running Ubuntu, you specifically need the packages
``ros-electric-navigation`` and ``ros-electric-visualization`` that should install
all required dependencies (but you still need the ROS Python 3 overlay, see the
installation notes linked above).

Setting up the scenario
-----------------------

For the navigation-stack to work, you need your robot (in our example the PR2
robot) equipped with the following components:

#. **Pose sensor** - posts the position of the robot on a ROS topic and will be
   read to create the TF-tree
#. **IMU** - posts the Velocity of the robot and will be read by ROS navigation 
#. **Motion controller** - receives speeds for translation and rotation of the
   robot from ROS navigation
#. **Laserscanner** - Perception of the robot for localization and costmaps of
   ROS navigation
#. **Joint state publisher** - Posts the current state of the joints of the
   robot (this is needed because we are also working with non-fixed joints
   here. If the URDF of your robot only has fixed joints, you will not need
   this component.)

There is already a completely configured builder-script in
``$MORSE_PREFIX/share/morse/examples/tutorials/ros_navigation/scenario.py``.

You can open this script in your favorite Python editor to have a look at it.
Note that the components in this tutorial are linked using the ROS middleware.

Open it then in MORSE with:

``morse edit
$MORSE_PREFIX/share/morse/examples/tutorials/ros_navigation/scenario.py``


Creating the ROS nodes
----------------------

.. note::
    All the nodes and launch files mentioned in this section are available
    in ``$MORSE_PREFIX/share/morse/examples/tutorials/ros_navigation``. If
    you wish to directly reuse these ROS nodes, do not forget to add this
    path to your ``$ROS_PACKAGE_PATH``.

From MORSE to the standard ROS navigation datastructures
++++++++++++++++++++++++++++++++++++++++++++++++++++++++

MORSE does not currently directly export a TF tree of the robot, which is
required by the navigation stack. Precisely, the ROS navigation stack expects a
TF tree of the robot, the map as well as the pose and speed of the robot.

The ``morse_tf`` package implements a TF-broadcaster that reads the values of
the MORSE Pose and IMU sensors and posts the messages and TF-frames on the
topic needed by the ROS navigation stack. It also sets the odom-TF frame to the
initial robot's starting position when starting the simulation. 

The complete TF-tree of the robot results from the combination of the odom
frame, the map frame and a URDF model of the robot, posted the robot state
publisher.

The URDF of the PR2 robot can be found in the ROS package ``morse_tf`` in the
folder ``urdf``. The robot state publisher also needs the jointstates of all
non-fixed joints of the robot which are exported by the ``pr2_posture``
component.

.. note::
    We are not going deep into the ROS navigation stack, TF and the robot
    state publisher here. It is strongly recommended to do the tutorials on that
    topic on http://www.ros.org!

The ROS navigation node
+++++++++++++++++++++++

We need to create a new ROS node to specify:

- the required dependencies (navigation, etc.: a *manifest* file)
- the launch procedure (a *launch* file)


The manifest
~~~~~~~~~~~~

.. code-block:: xml

    <package>
    <description brief="morse_2dnav">

        morse_2dnav is a sample ROS node used to demo
        2D planar navigation in the MORSE simulator.

    </description>
    <author>MORSE Team</author>
    <license>BSD</license>
    <review status="unreviewed" notes=""/>
    <url>http://morse.openrobots.org</url>
    <depend package="move_base"/>
    <depend package="morse_tf"/>
    <depend package="map_server"/>
    <depend package="robot_state_publisher"/>
    </package>

The manifest file mainly lists the package dependencies.

The launch file
~~~~~~~~~~~~~~~

.. code-block:: xml

    <launch>
    <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen" clear_params="true">
        <remap from="/base_scan" to="/pr2/Sick"/>
        <rosparam file="$(find morse_2dnav)/morse_move_base/costmap_common_params.yaml" command="load" ns="global_costmap" />
        <rosparam file="$(find morse_2dnav)/morse_move_base/costmap_common_params.yaml" command="load" ns="local_costmap" />
        <rosparam file="$(find morse_2dnav)/morse_move_base/local_costmap_params.yaml" command="load" />
        <rosparam file="$(find morse_2dnav)/morse_move_base/global_costmap_params.yaml" command="load" />
        <param name="base_local_planner" value="dwa_local_planner/DWAPlannerROS" />
        <rosparam file="$(find morse_2dnav)/launch/morse_move_base/dwa_planner_ros.yaml" command="load" />
    </node>

    <node name="map_server" pkg="map_server" type="map_server" args="$(find morse_2dnav)/maps/tut1_map.yaml"/> 

    <node pkg="morse_tf" type="morse_tf_pr2.py" name="morse_tf_pr2"/>

    <param name="robot_description" command="cat $(find morse_tf)/urdfs/pr2.urdf"/>

    <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher"> 
        <remap from="joint_states" to="/pr2/ptu_posture" />
    </node>
    </launch>

Starting the navigation
-----------------------

Now we can finally start our navigation-simulation

#. Hit ``p`` in MORSE to start the simulation
#. Start a ROS master node by typing ``roscore`` (This step is optional but
   recommended)
#. Type ``roslaunch morse_2dnav 2dnav.launch``. This should bring up all needed
   nodes and topics. 
#. You can now start RVIZ in a seperate terminal by ``rosrun rviz rviz`` and
   see if everything is fine by visualizing for example the map, laserscan,
   odometry, etc... There is also a default configuration for RVIZ that
   visualizes everything needed for navigation in the ``morse_2dnav``
   ROS-package in the folder ``rviz``.  By using "move_base_simple/goal" as 2D
   Nav Goal (you can edit the 2D Nav Goal in the Windows "Tool Properties"),
   you can set a navigation-goal the robot should navigate to by clicking on
   the map. Your robot should now start to navigate towards that point on the
   map.

If everything worked out fine, it should look something like this:

.. image:: ../../../media/morse_ros_navigation.png
   :align: center

Notes
+++++

The morse_2dnav package already includes a 2D gridmap of the environment. This
map has been generated by using the simulated SICK-laserscanner in MORSE and
ROS GMapping. Watch out for a tutorial soon.

If you have further questions or problems, don't hesitate too write on the mailing-list!
