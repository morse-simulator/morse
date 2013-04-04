import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.sensor
from morse.helpers.components import add_data

class GPS(morse.core.sensor.Sensor):
    """
    This sensor emulates a GPS, providing the exact coordinates in the
    Blender scene. The coordinates provided by the GPS are with respect
    to the origin of the Blender coordinate reference.
    """

    _name = "GPS"

    add_data('x', 0.0, "float", \
             'x coordinate of the sensor, in world coordinate, in meter')
    add_data('y', 0.0, "float", \
             'y coordinate of the sensor, in world coordinate, in meter')
    add_data('z', 0.0, "float", \
             'z coordinate of the sensor, in world coordinate, in meter')

    def __init__(self, obj, parent=None):
        """ Constructor method.
            Receives the reference to the Blender object.
            The second parameter should be the name of the object's parent. """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        logger.info('Component initialized, runs at %.2f Hz', self.frequency)


    def default_action(self):
        """ Main function of this component. """
        x = self.position_3d.x
        y = self.position_3d.y
        z = self.position_3d.z

        # Store the data acquired by this sensor that could be sent
        #  via a middleware.
        self.local_data['x'] = float(x)
        self.local_data['y'] = float(y)
        self.local_data['z'] = float(z)
