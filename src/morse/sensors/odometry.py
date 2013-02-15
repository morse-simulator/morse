import logging; logger = logging.getLogger("morse." + __name__)
from morse.helpers.morse_math import normalise_angle
import morse.core.sensor
import copy
from morse.helpers.components import add_data, add_level

class Odometry(morse.core.sensor.Sensor):
    """
    This sensor produces relative displacement with respect to the position and
    rotation in the previous Blender tick. It can compute too the position of the
    robot with respect to its original position, and the associated speed.

    The angles for yaw, pitch and roll are given in radians.

    .. note::
      This sensor always provides perfect data.
      To obtain more realistic readings, it is recommended to add modifiers.
        
      - **Noise modifier**: Adds random Gaussian noise to the data
      - **Odometry Noise modifier**: Simulate scale factor error and gyroscope drift
    """

    _name = "Odometry"
    _short_desc = "An odometry sensor that returns raw, partially integrated or fully integrated displacement information."

    add_level("raw", "morse.sensors.odometry.RawOdometry", doc = "raw odometry: only dS is exported")
    add_level("differential", None, doc = "differential odometry, corresponding to standard 'robot level' odometry")
    add_level("integrated", "morse.sensors.odometry.IntegratedOdometry", doc = "integrated odometry: absolution position is exported", default=True)

    add_data('dS', 0.0, "float","curvilign distance since last tick", level = "raw")
    add_data('dx', 0.0, "float","delta of X coordinate of the sensor", level = "differential")
    add_data('dy', 0.0, "float","delta of Y coordinate of the sensor", level = "differential")
    add_data('dz', 0.0, "float","delta of Z coordinate of the sensor", level = "differential")
    add_data('dyaw', 0.0, "float","delta of rotation angle with respect to the Z axis", level = "differential")
    add_data('dpitch', 0.0, "float","delta of rotation angle with respect to the Y axis", level = "differential")
    add_data('droll', 0.0, "float","delta of rotation angle with respect to the X axis", level = "differential")
    add_data('x', 0.0, "float","X coordinate of the sensor", level = "integrated")
    add_data('y', 0.0, "float","Y coordinate of the sensor", level = "integrated")
    add_data('z', 0.0, "float","Z coordinate of the sensor", level = "integrated")
    add_data('yaw', 0.0, "float","rotation angle with respect to the Z axis", level = "integrated")
    add_data('pitch', 0.0, "float","rotation angle with respect to the Y axis", level = "integrated")
    add_data('roll', 0.0, "float","rotation angle with respect to the X axis", level = "integrated")
    add_data('vx', 0.0, "float","linear velocity related to the X coordinate of the sensor", level = "integrated")
    add_data('vy', 0.0, "float","linear velocity related to the Y coordinate of the sensor", level = "integrated")
    add_data('vz', 0.0, "float","linear velocity related to the Z coordinate of the sensor", level = "integrated")
    add_data('wz', 0.0, "float","angular velocity related to the Z coordinate of the sensor", level = "integrated")
    add_data('wy', 0.0, "float","angular velocity related to the Y coordinate of the sensor", level = "integrated")
    add_data('wx', 0.0, "float","angular velocity related to the X coordinate of the sensor", level = "integrated")


    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        # Call the constructor of the parent class
        super(Odometry, self).__init__(obj, parent)

        self.original_pos = copy.copy(self.position_3d)

        self.previous_pos = self.original_pos.transformation3d_with(
                                                            self.position_3d)

        logger.info('Component initialized')


    def default_action(self):
        """ Compute the relative position and rotation of the robot

        The measurements are taken with respect to the previous position
        and orientation of the robot
        """
        # Compute the position of the sensor within the original frame
        current_pos = self.original_pos.transformation3d_with(self.position_3d)

        # Compute the difference in positions with the previous loop
        self.local_data['dx'] = current_pos.x - self.previous_pos.x
        self.local_data['dy'] = current_pos.y - self.previous_pos.y
        self.local_data['dz'] = current_pos.z - self.previous_pos.z

        # Compute the difference in orientation with the previous loop
        dyaw = current_pos.yaw - self.previous_pos.yaw
        dpitch = current_pos.pitch - self.previous_pos.pitch
        droll = current_pos.roll - self.previous_pos.roll
        self.local_data['dyaw'] = normalise_angle(dyaw)
        self.local_data['dpitch'] = normalise_angle(dpitch)
        self.local_data['droll'] = normalise_angle(droll)

        # Store the 'new' previous data
        self.previous_pos = current_pos


class RawOdometry(Odometry):

    def __init__(self, obj, parent=None):
        # Call the constructor of the parent class
        super(RawOdometry, self).__init__(obj, parent)

    def default_action(self):
        # Compute the position of the sensor within the original frame
        current_pos = self.original_pos.transformation3d_with(self.position_3d)

        # Compute the difference in positions with the previous loop
        self.local_data['dS'] = current_pos.distance(self.previous_pos)

        # Store the 'new' previous data
        self.previous_pos = current_pos

class IntegratedOdometry(Odometry):

    def __init__(self, obj, parent=None):
        # Call the constructor of the parent class
        super(IntegratedOdometry, self).__init__(obj, parent)

    def default_action(self):
        # Compute the position of the sensor within the original frame
        current_pos = self.original_pos.transformation3d_with(self.position_3d)

        # Integrated version
        self.local_data['x'] = current_pos.x
        self.local_data['y'] = current_pos.y
        self.local_data['z'] = current_pos.z
        self.local_data['yaw'] = current_pos.yaw
        self.local_data['pitch'] = current_pos.pitch
        self.local_data['roll'] = current_pos.roll

        # speed in the sensor frame, related to the robot pose
        self.delta_pos = self.previous_pos.transformation3d_with(current_pos)
        self.local_data['vx'] = self.delta_pos.x * self.frequency
        self.local_data['vy'] = self.delta_pos.y * self.frequency
        self.local_data['vz'] = self.delta_pos.z * self.frequency
        self.local_data['wz'] = self.delta_pos.yaw * self.frequency
        self.local_data['wy'] = self.delta_pos.pitch * self.frequency
        self.local_data['wx'] = self.delta_pos.roll * self.frequency

        # Store the 'new' previous data
        self.previous_pos = current_pos

