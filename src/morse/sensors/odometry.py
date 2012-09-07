import logging; logger = logging.getLogger("morse." + __name__)
from morse.helpers.math import normalise_angle
import morse.core.sensor
import copy

class OdometryClass(morse.core.sensor.MorseSensorClass):
    """ Odometer sensor """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        self.original_pos = copy.copy(self.position_3d)

        self.previous_pos = self.original_pos.transformation3d_with(
                                                            self.position_3d)

        # Variables to store the previous status of the robot
        for i in ['dS', 'dx', 'dy', 'dz', 'dyaw', 'dpitch', 'droll',
                  'x', 'y', 'z', 'yaw', 'pitch', 'roll',
                  'vx', 'vy', 'vz', 'wz', 'wy', 'wx']:
            self.local_data[i] = 0.0

        logger.info('Component initialized')


    def default_action(self):
        """ Compute the relative position and rotation of the robot

        The measurements are taken with respect to the previous position
        and orientation of the robot
        """
        # Compute the position of the sensor within the original frame
        current_pos = self.original_pos.transformation3d_with(self.position_3d)

        # Compute the difference in positions with the previous loop
        self.local_data['dS'] = current_pos.distance(self.previous_pos)

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

        # Integrated version
        self.local_data['x'] = current_pos.x
        self.local_data['y'] = current_pos.y
        self.local_data['z'] = current_pos.z
        self.local_data['yaw'] = current_pos.yaw
        self.local_data['pitch'] = current_pos.pitch
        self.local_data['roll'] = current_pos.roll

        # speed in the sensor frame, related to the initial pose
        self.local_data['vx'] = self.local_data['dx'] / self.frequency
        self.local_data['vy'] = self.local_data['dy'] / self.frequency
        self.local_data['vz'] = self.local_data['dz'] / self.frequency
        self.local_data['wz'] = dyaw / self.frequency
        self.local_data['wy'] = dpitch / self.frequency
        self.local_data['wx'] = droll / self.frequency

        # Store the 'new' previous data
        self.previous_pos = current_pos
