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

        # Tick rate is the real measure of time in Blender.
        # By default it is set to 60, regardless of the FPS
        # If logic tick rate is 60, then: 1 second = 60 ticks
        self.ticks = bge.logic.getLogicTicRate()
        
        # The actual frequency at which the sensor action is called
        # When a delay of the sensor is set via frequency,
        # the action is not called for every logic tick.
        # frequency of the game sensor specifies how many actions are skipped
        # e.g. game sensor freq = 0 -> sensor runs at full logic rate
        self.freq = self.ticks / (self.blender_obj.sensors[0].frequency + 1)

        # The robot needs a physics controller!
        # Since the imu does not have physics,
        # make new references to the robot velocities and use those.
        self.robot_w = self.robot_parent.blender_obj.localAngularVelocity
        self.robot_vel = self.robot_parent.blender_obj.worldLinearVelocity

        # previous linear velocity
        self.plv = mathutils.Vector((0.0, 0.0, 0.0))

        # get gravity from scene?
        #g = bpy.data.scenes[0].game_settings.physics_gravity
        g = 9.81
        self.gravity = mathutils.Vector((0.0, 0.0, g))

        # get the transformation from robot to sensor frame
        (loc, rot, scale) = self.robot_parent.position_3d.transformation3d_with(self.position_3d).matrix.decompose()
        #logger.debug("rotation [%.4f %.4f %.4f]" % tuple(math.degrees(a) for a in rot.to_euler()))
        # store body to imu rotation and translation
        self.rot_b2i = rot
        self.trans_b2i = loc
        
        # reference for world to imu orientation
        self.rot_w2i = self.blender_obj.worldOrientation

        self.local_data['angular_velocity'] = [0.0, 0.0, 0.0]
        self.local_data['linear_acceleration'] = [0.0, 0.0, 0.0]

        logger.info("IMU Component initialized, runs at %.2f Hz" % self.freq)


    def default_action(self):
        """ Get the speed and acceleration of the robot and transform it into the imu frame """

        rates = self.rot_b2i * self.robot_w
        #logger.debug("rates in robot frame (%.4f, %.4f, %.4f)" % tuple(self.robot_w))
        #logger.debug("rates in imu frame (%.4f, %.4f, %.4f)" % tuple(rates))

        # differentiate linear velocity in world (inertial) frame
        # and rotate to imu frame
        dv_imu = self.rot_w2i * (self.robot_vel - self.plv) * self.freq
        #logger.debug("velocity_dot in imu frame (%.4f, %.4f, %.4f)" % tuple(dv_imu))
        
        # rotate acceleration due to gravity into imu frame
        g_imu = self.rot_w2i * self.gravity
        
        # acceleration due to rotation
        # is zero if imu is mounted in robot center (assumed axis of rotation)
        a_rot = rates.cross(rates.cross(self.trans_b2i))
        #logger.debug("accel rot (%.4f, %.4f, %.4f)" % tuple(a_rot))
        
        # final measurement includes gravity and acceleration 
        accel_meas = dv_imu + g_imu + a_rot

        # save velocity for next step
        self.plv = self.robot_vel.copy()

        # Store the important data
        self.local_data['angular_velocity'][0] = rates[0]
        self.local_data['angular_velocity'][1] = rates[1]
        self.local_data['angular_velocity'][2] = rates[2]

        self.local_data['linear_acceleration'][0] = accel_meas[0]
        self.local_data['linear_acceleration'][1] = accel_meas[1]
        self.local_data['linear_acceleration'][2] = accel_meas[2]
