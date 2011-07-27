ROS-navigation tutorial (experimental)
======================================

This tutorial shows how to use the ROS navigation stack to navigate a robot in MORSE.

Setup
-----

You need to have a working installation of ROS Diamondback and also have the python3-compatible stacks for MORSE-ROS installed. You can find
information about this here:  :doc:`installation notes <../installation>`

You should also be familiar with the basic usage of ROS and how to use TF and the ROS navigation stack. You should also know about launchfiles and topic remapping as well as the robot state publisher. Also experience with RVIZ are of advantage. Tutorials on all of those topics can be found on http://www.ros.org.

To create the TF-tree, you have to write a TF-broadcaster that build a TF-tree from informations about the robot`s pose, velocity and the robot model (in ROS usually given as URDF-file). The TF-tree also need information about the current configuration of the joints of the robot in case they are not fixed joints. You also need to set the parameters of the ROS navigation (local planner, move_base, etc). Detailed information about the ROS navigation-stack can be found here: http://www.ros.org/wiki/navigation.

You can download a ROS-stack including an example implementation for using ROS navigation with the simulated Jido robot in MORSE in the following way:

First create a folder for your MORSE-ROS stack and make sure, it is in your ROS_PACKAGE_PATH:

``mkdir morse_ros``

``cd morse_ros``

Now init a git repository and checkout the morse_ros stack for the navigation tutorial:

``git init``

``git remote add -t tutorials -f origin git://github.com/kargm/morse_ros.git``

``git checkout tutorials``

If you now type ``roscd morse_ros``, you should get to your morse_ros-stack. If you don´t, the morse-ros stack is NOT in your ROS_PACKAGE_PATH.

Finally rosmake the stack: 

``rosmake morse_ros``

Configuring the scenario
------------------------

There is already a completely configured scenario in ``examples/morse/scenarii/ROS_tutorial1_navstack.blend``. Open this file in MORSE.

For the navigation-stack to work, you need your robot (in our example the Jido robot) equipped with the following components:

#. **Pose sensor** - posts the position of the robot on a ROS topic and will be read to create the TF-tree
#. **IMU** - posts the Velocity of the robot and will be read by ROS navigation 
#. **Motion controller** - receives speeds for translation and rotation of the robot from ROS navigation
#. **Laserscanner** - Perception of the robot for localization and costmaps of ROS navigation
#. **Joint state publisher** - Posts the current state of the joints of the robot (this is needed because we are also working with non-fixed joints here. If the URDF of your robot only has fixed joints, you will not need this component.)

More information about how to equip your robot with those components can be found in the MORSE beginner tutorial: :doc:`Building an equipped robot  <./equip_robot>`. The components in this tutorial are linked using the ROS middleware. 

Linking the MORSE-ROS topics with the ROS navigation stack.
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

The ROS navigation stack expects a tf-tree of the robot and the map as well as the pose of the robot and its speed. The morse_ros-package includes a tf-broadcaster that reads the values of the values of the Pose Sensor and the IMU sensor and posts the messages and TF-frames on the topic needed by the ROS navigation stack. It also sets the odom-TF-frame to the initial robot's starting position when starting the Simulation. The TF-tree of the robot is created by the odom frame, the map frame and a urdf of the robot posted the robot state publisher. The URDF of the Jido robot can be found in the ROS package ``morse_tf`` in the folder ``urdf``. The robot state publisher also needs the jointstates of the robot`s  flexible Joints which are exported by the ``jido_posture`` component.

NOTE: We are not going to deep into the ROS navigation stack, TF and the robot state publisher here. It is strongly recommened to do the tutorials on that topic on http://www.ros.org! 

Starting the navigation
+++++++++++++++++++++++

Now we can finally start our navigation-simulation

#. Hit ``P`` in MORSE to start the simulation
#. Start a roscore by typing ``roscore`` (This step is optional but recommended)
#. Type ``roslaunch morse_2dnav 2dnav_movebase.launch``. This should bring up all needed nodes and topics. 
#. You can now start RVIZ in a seperate terminal by ``rosrun rviz rviz`` and see if everything is fine by visualizing for example the map, laserscan, odometry, etc... There is also a default configuration for RVIZ that visualizes everything needed for navigation in the ``morse_2dnav`` ROS-package in the folder ``rviz``.  By using "move_base_simple/goal" as 2D Nav Goal (you can edit the 2D Nav Goal in the Windows "Tool Properties"), you can set a navigation-goal the robot should navigate to by clicking on the map. Your robot should now start to navigate towards that point on the map.

If everything worked out fine, it should look something like this:

.. image:: ../../../media/morse_ros_navigation.png
   :align: center

Notes
+++++

The morse_2dnav package already includes a 2D gridmap of the environment. This map has been generated by using the simulated SICK-laserscanner in MORSE and ROS GMapping. Watch out for a tutorial soon.

If you have further questions or problems, don`t hesitate too write on the mailing-list!
