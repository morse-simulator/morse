import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.sensor

class Clock(morse.core.sensor.Sensor):
    """ 
    This sensor returns the current time, measured by the robot (in ms).
    """
    _name = "Clock"

    def __init__(self, obj, parent=None):
        """ Constructor method.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)

        logger.info('Component initialized, runs at %.2f Hz', self.frequency)


    def default_action(self):
        """
        Do nothing, we are only interested in the 'timestamp'
        automatically put by sensor.
        """
        pass
