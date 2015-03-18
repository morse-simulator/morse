import logging; logger = logging.getLogger("morse." + __name__)
import math
import morse.core.sensor
from morse.helpers.components import add_data, add_property
from morse.core.mathutils import *

class Accelerometer(morse.core.sensor.Sensor):
    """ 
    This sensor emulates an Accelerometer/Podometer, measuring the
    distance that a robot has moved, the current speed and current
    acceleration. Measurements are done for the 3 axes (X, Y, Z) for
    velocity and acceleration. The values for velocity and acceleration
    are measured at each tic of the Game Engine, measuring the
    difference in distance from the previous tic, and the estimated time
    between tics (60 tics per second is the default in Blender).
    """

    _name = "Accelerometer"

    add_data('distance', 0.0, "float", 
             'distance travelled since the last tick, in meter')
    add_data('velocity', [0.0, 0.0, 0.0], "vec3<float>", 
             'Instantaneous speed in X, Y, Z, in meter sec^-1')
    add_data('acceleration', [0.0, 0.0, 0.0], "vec3<float>", 
             'Instantaneous acceleration in X, Y, Z, in meter sec^-2')
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

        self.pp = Vector((0.0, 0.0, 0.0)) # previous position
        self.plv = Vector((0.0, 0.0, 0.0)) # previous linear velocity
        self.pav = Vector((0.0, 0.0, 0.0)) # previous angular velocity
        self.dp = Vector((0.0, 0.0, 0.0))  # diff position
        self.pt = 0.0 # previous timestamp
        self.dt = 0.0 # diff

        self.v = Vector((0.0, 0.0, 0.0)) # current linear velocity
        self.a = Vector((0.0, 0.0, 0.0)) # current angular velocity

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


        # imu2body will transform a vector from imu frame to body frame
        self.imu2body = self.sensor_to_robot_position_3d()
        # rotate vector from body to imu frame
        self.rot_b2i = self.imu2body.rotation.conjugated()

        if self.imu2body.translation.length > 0.01:
            self.compute_offset_acceleration = True
        else:
            self.compute_offset_acceleration = False

        if self._type == 'Velocity':
            # make new references to the robot velocities and use those.
            self.robot_w = self.robot_parent.bge_object.localAngularVelocity
            self.robot_vel = self.robot_parent.bge_object.worldLinearVelocity

        logger.info('Component initialized, runs at %.2f Hz', self.frequency)

    def _sim_simple(self):
        self.v = self.dp / self.dt
        self.a = (self.v - self.plv) / self.dt

        # Update the data for the velocity
        self.plv = self.v.copy()

        # Store the important data
        w2a = self.position_3d.rotation_matrix.transposed()
        self.local_data['velocity'] = w2a * self.v
        self.local_data['acceleration'] = w2a * self.a

    def _sim_physics(self):
        w2a = self.position_3d.rotation_matrix.transposed()
        # rotate the angular rates from the robot frame into the imu frame
        rates = self.rot_b2i * self.robot_w

        # differentiate linear velocity in world (inertial) frame
        # and rotate to imu frame
        self.a = w2a * (self.robot_vel - self.plv) / self.dt

        if self.compute_offset_acceleration:
            # acceleration due to rotation (centripetal)
            # is zero if imu is mounted in robot center (assumed axis of rotation)
            a_centripetal = self.rot_b2i * rates.cross(rates.cross(self.imu2body.translation))
            #logger.debug("centripetal acceleration (% .4f, % .4f, % .4f)", a_rot[0], a_rot[1], a_rot[2])

            # linear acceleration due to angular acceleration
            a_alpha = self.rot_b2i * (self.robot_w - self.pav).cross(self.imu2body.translation) / self.dt

            # final measurement includes acceleration due to rotation center not in IMU
            self.a += a_centripetal + a_alpha

        # save velocity for next step
        self.plv = self.robot_vel.copy()
        self.pav = self.robot_w.copy()

        self.local_data['velocity'] = w2a * self.robot_vel
        self.local_data['acceleration'] = self.a

    def default_action(self):
        """ Compute the speed and accleration of the robot """
        # Compute the difference in positions with the previous loop
        self.dt = self.robot_parent.gettime() - self.pt
        self.pt = self.robot_parent.gettime()

        if self.dt < 1e-6:
            return

        self.dp = self.position_3d.translation - self.pp
        self.local_data['distance'] = \
            math.sqrt(self.dp[0]**2 + self.dp[1]**2 + self.dp[2]**2)
        logger.debug("DISTANCE: %.4f" % self.local_data['distance'])

        # Store the position in this instant
        self.pp = self.position_3d.translation

        if self._type == 'Velocity':
            self._sim_physics()
        else:
            self._sim_simple()

