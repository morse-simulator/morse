import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.sensor
from morse.helpers.components import add_data, add_property
from morse.core.mathutils import * 
from math import degrees
from morse.helpers.velocity import linear_velocities, angular_velocities
from copy import copy

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

    add_property('_type', 'Automatic', 'ComputationMode', 'string',
                 "Kind of computation, can be one of ['Velocity', 'Position']. "
                 "Only robot with dynamic and Velocity control can choose Velocity "
                 "computation. Default choice is Velocity for robot with physics, "
                 "and Position for others")

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)

        self.pp = copy(self.position_3d)
        self.pt = 0.0 # previous timestamp
        self.dt = 0.0 # diff

        has_physics = bool(self.robot_parent.bge_object.getPhysicsId())
        if self._type == 'Automatic':
            if has_physics: 
                self._type = 'Velocity'
            else:
                self._type = 'Position'

        if self._type == 'Velocity' and not has_physics:
            logger.error("Invalid configuration : Velocity computation without "
                        "physics")
            return


        # make new references to the robot velocities and use those.
        self.robot_w = self.robot_parent.bge_object.localAngularVelocity
        self.robot_v = self.robot_parent.bge_object.localLinearVelocity
        self.robot_world_v = self.robot_parent.bge_object.worldLinearVelocity

        # get the quaternion which will rotate a vector from body to sensor frame
        self.rot_b2s = self.sensor_to_robot_position_3d().rotation.conjugated()
        logger.debug("body2sensor rotation RPY [% .3f % .3f % .3f]" %
                     tuple(degrees(a) for a in self.rot_b2s.to_euler()))

        logger.info("Component initialized, runs at %.2f Hz", self.frequency)


    def _sim_simple(self):
        self.dt = self.robot_parent.gettime() - self.pt
        self.pt = self.robot_parent.gettime()

        if self.dt < 1e-6:
            return

        v = linear_velocities(self.pp, self.position_3d, self.dt)
        w = angular_velocities(self.pp, self.position_3d, self.dt)

        self.pp = copy(self.position_3d)

        w2a = self.position_3d.rotation_matrix.transposed()

        self.local_data['linear_velocity'] = w2a * v
        self.local_data['angular_velocity'] =  w
        self.local_data['world_linear_velocity'] = v

    def default_action(self):
        """ Get the linear and angular velocity of the blender object. """

        if self._type == 'Velocity':
            # Store the important data
            self.local_data['linear_velocity'] = self.rot_b2s * self.robot_v
            self.local_data['angular_velocity'] = self.rot_b2s * self.robot_w
            self.local_data['world_linear_velocity'] = self.robot_world_v.copy()
        else:
            self._sim_simple()
