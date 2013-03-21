import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.sensor
from morse.helpers.components import add_data

class Pose(morse.core.sensor.Sensor):
    """ 
    This sensor returns the full pose of the sensor, i.e. both
    translation and rotation with respect to the Blender world frame.
    """
    _name = "Pose"

    add_data('x', 0.0, "float", \
             'x coordinate of the sensor, in world coordinate, in meter')
    add_data('y', 0.0, "float", \
             'y coordinate of the sensor, in world coordinate, in meter')
    add_data('z', 0.0, "float", \
             'z coordinate of the sensor, in world coordinate, in meter')
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
        """ Get the x, y, z, yaw, pitch and roll of the blender object. """
        self.local_data['x'] = self.position_3d.x
        self.local_data['y'] = self.position_3d.y
        self.local_data['z'] = self.position_3d.z
        self.local_data['yaw'] = self.position_3d.yaw
        self.local_data['pitch'] = self.position_3d.pitch
        self.local_data['roll'] = self.position_3d.roll
