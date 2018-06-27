Using URDF robot description
============================

.. warning::

    URDF support is currently considered experimental. Please report any issue
    you encounter on `the MORSE issue
    tracker <https://github.com/morse-simulator/morse/issues>`_.

`URDF <http://wiki.ros.org/urdf>`_ is a standard XML format used to describe
robots. You can load URDF files directly from MORSE :doc:`builder
script<user/builder>` by simply passing a path to a URDF file to a robot
constructor:

.. code-block:: python

    from morse.builder import *

    # Add robot from its URDF description
    robot = Robot("path/to/my/robot.urdf", name="MyRobot")

    # then, standard BUilder script. For instance:

    # Append an actuator to the robot
    motion = MotionVW()
    robot.append(motion)

    # Append a sensor to the robot
    pose = Pose()
    pose.translate(z = 0.75)
    robot.append(pose)

    # Configure the robot on the 'socket' interface
    robot.add_default_interface('socket')

    env = Environment('indoors-1/indoor-1')


MORSE will automatically import the Collada/STL meshes referenced in your URDF
file.

.. warning::

    Often, the URDF file references meshes using the ROS specific `package://`
    protocol. In this case, MORSE attempts to find the meshes by replacing
    `package://` by the first entry in your `$ROS_PACKAGE_PATH` environment
    variable. If MORSE fails to find the meshes, check your `$ROS_PACKAGE_PATH`.

    Alternatively, provide full paths to your meshes in your URDF file using the
    `file://` protocol.
