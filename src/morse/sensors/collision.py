import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.sensor import Sensor
from morse.helpers.components import add_data, add_property
from morse.core import blenderapi

class Collision(Sensor):
    """
    Sensor to detect objects colliding with the current object,
    with more settings than the Touch sensor
    """
    _name = "Collision"
    _short_desc = "Detect objects colliding with the current object."

    add_data('collision', False, "bool", "objects colliding with the current object")
    add_data('objects', "", "string", "A list of colliding objects.")

    # These properties are not used directly in the logic, but are used
    # in the builder to create the radar properly.
    # These value cannot be changed dynamically in bge.
    add_property('_collision_property', "", 'collision_property', 'string',
                 'Only look for objects with this property, '
                 'default "" (all objects)')

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
