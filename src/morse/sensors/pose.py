import logging; logger = logging.getLogger("morse." + __name__)
import GameLogic
import math
import morse.core.sensor

class PoseClass(morse.core.sensor.MorseSensorClass):
    """ Robot pose sensor, including position and orientation """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self.local_data['x'] = 0.0
        self.local_data['y'] = 0.0
        self.local_data['z'] = 0.0
        self.local_data['yaw'] = 0.0
        self.local_data['pitch'] = 0.0
        self.local_data['roll'] = 0.0

        logger.info('Component initialized')


    def default_action(self):
        """ Get the x, y, z, yaw, pitch and roll of the blender object. """
        x = self.position_3d.x
        y = self.position_3d.y
        z = self.position_3d.z

        yaw = self.position_3d.yaw
        pitch = self.position_3d.pitch
        roll = self.position_3d.roll

        # Store the orientation values in the robot's object
        self.robot_parent.yaw = yaw
        self.robot_parent.pitch = pitch
        self.robot_parent.roll = roll

        # Store the data acquired by this sensor that could be sent
        #  via a middleware.
        self.local_data['x'] = float(x)
        self.local_data['y'] = float(y)
        self.local_data['z'] = float(z)
        # Store the data acquired by this sensor that could be sent
        #  via a middleware.
        self.local_data['yaw'] = float(yaw)
        self.local_data['pitch'] = float(pitch)
        self.local_data['roll'] = float(roll)
