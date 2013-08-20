import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.sensor import Sensor
from morse.helpers.components import add_data
from morse.core import blenderapi

class Collision(Sensor):
    """ Sensor to detect objects colliding with the current object,
    with more settings than the Touch sensor """
    _name = "Collision"

    add_data('collision', False, "bool", "objects colliding with the current object")
    add_data('objects', "", "string", "A list of colliding objects.")

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        Sensor.__init__(self, obj, parent)

        logger.info('Component initialized, runs at %.2f Hz', self.frequency)

    def default_action(self):
        """ Is currently in collision """
        controller  = blenderapi.controller()
        sensor = controller.sensors[-1]
        # see hitObjectList and hitObject for last collided object(s)
        self.local_data['collision'] = sensor.positive
        self.local_data['objects'] = ','.join([o.name for o in sensor.hitObjectList])
        # logger.debug(self.local_data['objects'])
