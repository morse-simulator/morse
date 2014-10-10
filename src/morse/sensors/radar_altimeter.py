import logging
logger = logging.getLogger("morse." + __name__)
from morse.core.sensor import Sensor
from morse.helpers.components import add_data, add_property

class RadarAltimeter(Sensor):
    """
    Sensor to compute the distance to the ground

    To work properly, the ground and relevant obstacles must be marked as Actor.
    """
    _name = "Radar Altimeter"
    _short_desc = "Compute the distance to the ground"

    add_data('z', 0.0, "float", 'Distance to "ground"')
    add_property('_max_range', 30.0, "MaxRange", "float", 
                 "Maximum distance to which ground is detected."
                 "If nothing is detected, return +infinity")

    def __init__(self, obj, parent=None):
        """ Constructor method.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        Sensor.__init__(self, obj, parent)

        logger.info('Component initialized, runs at %.2f Hz', self.frequency)

    def default_action(self):
        """ 
        Send a laser directly underneath and check the position of what it hits.
        """
        target = self.position_3d.translation
        target[2] -= 1.0

        _, point, _ = self.bge_object.rayCast(target, None, self._max_range)
        logger.debug("RadarAltimeter points to %s and hits %s" % (target, point))
        if point:
            self.local_data['z'] = self.bge_object.getDistanceTo(point)
        else:
            self.local_data['z'] = float('inf')
