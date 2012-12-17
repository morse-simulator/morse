import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.sensor

class GPSClass(morse.core.sensor.Sensor):
    """ Class definition for the gyroscope sensor.
        Sub class of Morse_Object. """

    def __init__(self, obj, parent=None):
        """ Constructor method.
            Receives the reference to the Blender object.
            The second parameter should be the name of the object's parent. """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self.local_data['x'] = 0.0
        self.local_data['y'] = 0.0
        self.local_data['z'] = 0.0

        logger.info('Component initialized')


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
