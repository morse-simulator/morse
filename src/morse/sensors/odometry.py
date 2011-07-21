import logging; logger = logging.getLogger("morse." + __name__)
import GameLogic
import math
import morse.core.sensor

class OdometryClass(morse.core.sensor.MorseSensorClass):
    """ Odometer sensor """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        # Variables to store the previous status of the robot
        self.previous_position = [self.position_3d.x, self.position_3d.y, self.position_3d.z]
        self.previous_orientation = [self.position_3d.yaw, self.position_3d.pitch, self.position_3d.roll]

        self.local_data['dx'] = 0.0
        self.local_data['dy'] = 0.0
        self.local_data['dz'] = 0.0
        self.local_data['dyaw'] = 0.0
        self.local_data['dpitch'] = 0.0
        self.local_data['droll'] = 0.0

        logger.info('Component initialized')


    def default_action(self):
        """ Compute the relative position and rotation of the robot

        The measurements are taken with respect to the previous position
        and orientation of the robot
        """
        # Compute the difference in positions with the previous loop
        self.local_data['dx'] = self.robot_parent.position_3d.x - self.previous_position[0]
        self.local_data['dy'] = self.robot_parent.position_3d.y - self.previous_position[1]
        self.local_data['dz'] = self.robot_parent.position_3d.z - self.previous_position[2]

        # Compute the difference in orientation with the previous loop
        self.local_data['dyaw'] = self.robot_parent.position_3d.yaw - self.previous_orientation[0]
        self.local_data['dpitch'] = self.robot_parent.position_3d.pitch - self.previous_orientation[1]
        self.local_data['droll'] = self.robot_parent.position_3d.roll - self.previous_orientation[2]

        # Store the 'new' previous data
        self.previous_position = [self.robot_parent.position_3d.x, self.robot_parent.position_3d.y, self.robot_parent.position_3d.z]
        self.previous_orientation = [self.robot_parent.position_3d.yaw, self.robot_parent.position_3d.pitch, self.robot_parent.position_3d.roll]

