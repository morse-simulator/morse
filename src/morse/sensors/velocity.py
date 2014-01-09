import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.sensor
from morse.helpers.components import add_data
from math import degrees

class Velocity(morse.core.sensor.Sensor):
    """
    This sensor returns the linear and angular velocity of the sensor,
    both in robot frame and in world frame. Linear velocities are
    expressed in meter . sec ^ -1 while angular velocities are expressed
    in radian . sec ^ -1.

    The sensor expects that the associated robot has a physics controller.
    """

    _name = "Velocity"
    _short_descr = "A Velocity Sensor"

    add_data('linear_velocity', [0.0, 0.0, 0.0], "vec3<float>",
             'velocity in sensor x, y, z axes (in meter . sec ^ -1)')
    add_data('angular_velocity', [0.0, 0.0, 0.0], "vec3<float>",
             'rates in sensor x, y, z axes (in radian . sec ^ -1)')
    add_data('world_linear_velocity', [0.0, 0.0, 0.0], "vec3<float>",
             'velocity in world x, y, z axes (in meter . sec ^ -1)')

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)

        # The robot needs a physics controller!
        # Since the sensor does not have physics
        if not bool(self.robot_parent.bge_object.getPhysicsId()):
            logger.error("The robot doesn't have a physics controller!")

        # make new references to the robot velocities and use those.
        self.robot_w = self.robot_parent.bge_object.localAngularVelocity
        self.robot_v = self.robot_parent.bge_object.localLinearVelocity
        self.robot_world_v = self.robot_parent.bge_object.worldLinearVelocity

        # get the quaternion which will rotate a vector from body to sensor frame
        self.rot_b2s = self.sensor_to_robot_position_3d().rotation.conjugated()
        logger.debug("body2sensor rotation RPY [% .3f % .3f % .3f]" %
                     tuple(degrees(a) for a in self.rot_b2s.to_euler()))

        logger.info("Component initialized, runs at %.2f Hz", self.frequency)


    def default_action(self):
        """ Get the linear and angular velocity of the blender object. """

        # Store the important data
        self.local_data['linear_velocity'] = self.rot_b2s * self.robot_v
        self.local_data['angular_velocity'] = self.rot_b2s * self.robot_w
        self.local_data['world_linear_velocity'] = self.robot_world_v.copy()
