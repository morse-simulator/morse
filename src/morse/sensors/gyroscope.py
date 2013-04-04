import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.sensor
from morse.helpers.components import add_data

class Gyroscope(morse.core.sensor.Sensor):
    """
    This sensor emulates a Gyroscope, providing the yaw, pitch and roll
    angles of the sensor object with respect to the Blender world
    reference axes.

    Angles are given in radians.
    """

    _name = "Gyroscope"

    add_data('yaw', 0.0, "float", \
             'rotation around the Z axis of the sensor, in radian')
    add_data('pitch', 0.0, "float", \
             'rotation around the Y axis of the sensor, in radian')
    add_data('roll', 0.0, "float", \
             'rotation around the X axis of the sensor, in radian')

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        logger.info('Component initialized, runs at %.2f Hz', self.frequency)


    def default_action(self):
        """ Get the yaw, pitch and roll of the blender object. """
        yaw = self.position_3d.yaw
        pitch = self.position_3d.pitch
        roll = self.position_3d.roll

        # Store the values in the robot's object
        self.robot_parent.yaw = yaw
        self.robot_parent.pitch = pitch
        self.robot_parent.roll = roll

        # Store the data acquired by this sensor that could be sent
        #  via a middleware.
        self.local_data['yaw'] = float(yaw)
        self.local_data['pitch'] = float(pitch)
        self.local_data['roll'] = float(roll)
