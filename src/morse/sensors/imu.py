import logging; logger = logging.getLogger("morse." + __name__)
import bge
import math
import mathutils
import morse.core.sensor

"""
Important note:

    The 'logger.debug' instructions take some processor work, even if they are
    not displayed. For this reason, it is best to comment out these lines in
    the 'default_action' method.
"""

class ImuClass(morse.core.sensor.MorseSensorClass):
    """ IMU sensor """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        #logger.setLevel(logging.DEBUG)

        # The robot needs a physics controller!
        # Since the imu does not have physics,
        # make new references to the robot velocities and use those.
        self.robot_w = self.robot_parent.blender_obj.localAngularVelocity
        self.robot_vel = self.robot_parent.blender_obj.worldLinearVelocity

        # previous linear velocity
        self.plv = mathutils.Vector((0.0, 0.0, 0.0))
        # previous angular velocity
        self.pav = mathutils.Vector((0.0, 0.0, 0.0))

        # get gravity from scene?
        #g = bpy.data.scenes[0].game_settings.physics_gravity
        g = 9.81
        self.gravity = mathutils.Vector((0.0, 0.0, g))

        # get the transformation from robot to sensor frame
        (loc, rot, scale) = self.robot_parent.position_3d.transformation3d_with(self.position_3d).matrix.decompose()
        logger.debug("body2imu rotation RPY [% .3f % .3f % .3f]" % tuple(math.degrees(a) for a in rot.to_euler()))
        logger.debug("body2imu translation [% .3f % .3f % .3f]" % tuple(loc))
        # store body to imu rotation and translation
        self.rot_b2i = rot
        self.trans_b2i = loc
        
        if (loc.length > 0.01):
            self.compute_offset_acceleration = True
        else:
            self.compute_offset_acceleration = False

        # reference for rotating world frame to imu frame
        self.rot_w2i = self.blender_obj.worldOrientation

        self.local_data['angular_velocity'] = [0.0, 0.0, 0.0]
        self.local_data['linear_acceleration'] = [0.0, 0.0, 0.0]

        logger.info("IMU Component initialized, runs at %.2f Hz ", self.frequency)


    def default_action(self):
        """ Get the speed and acceleration of the robot and transform it into the imu frame """

        # rot_b2i rotates body frame to imu frame
        # take the inverse rotation to transform a vector from body to imu
        rates = self.rot_b2i.conjugated() * self.robot_w
        #logger.debug("rates in robot frame (% .4f, % .4f, % .4f)", self.robot_w[0], self.robot_w[1], self.robot_w[2])
        #logger.debug("rates in imu frame   (% .4f, % .4f, % .4f)", rates[0], rates[1], rates[2])

        # differentiate linear velocity in world (inertial) frame
        # and rotate to imu frame
        dv_imu = self.rot_w2i.transposed() * (self.robot_vel - self.plv) * self.frequency
        #logger.debug("velocity_dot in imu frame (% .4f, % .4f, % .4f)", dv_imu[0], dv_imu[1], dv_imu[2])

        # rotate acceleration due to gravity into imu frame
        g_imu = self.rot_w2i.transposed() * self.gravity

        # measurement includes gravity and acceleration 
        accel_meas = dv_imu + g_imu

        if self.compute_offset_acceleration:
            # acceleration due to rotation (centripetal)
            # is zero if imu is mounted in robot center (assumed axis of rotation)
            a_centripetal = self.rot_b2i.conjugated() * rates.cross(rates.cross(self.trans_b2i))
            #logger.debug("centripetal acceleration (% .4f, % .4f, % .4f)", a_rot[0], a_rot[1], a_rot[2])

            # linear acceleration due to angular acceleration
            a_alpha = self.rot_b2i.conjugated() * (self.robot_w - self.pav).cross(self.trans_b2i) * self.frequency

            # final measurement includes acceleration due to rotation center not in IMU
            accel_meas += a_centripetal + a_alpha

        # save velocity for next step
        self.plv = self.robot_vel.copy()
        self.pav = self.robot_w.copy()

        # Store the important data
        self.local_data['angular_velocity'][0] = rates[0]
        self.local_data['angular_velocity'][1] = rates[1]
        self.local_data['angular_velocity'][2] = rates[2]

        self.local_data['linear_acceleration'][0] = accel_meas[0]
        self.local_data['linear_acceleration'][1] = accel_meas[1]
        self.local_data['linear_acceleration'][2] = accel_meas[2]
