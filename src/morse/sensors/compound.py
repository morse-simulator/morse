import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.sensor
from morse.core import blenderapi
from morse.helpers.components import add_property

class CompoundSensor(morse.core.sensor.Sensor):
    """
    This special sensor is constructed by passing a list of other sensors, and
    creates a new datastream from the concatenation of other sensors'
    `local_data`.

    More accurately, it streams a dictionary of `{<sensor name>: <sensor
    local_data>}`.

    Note that services exposed by original sensors *are not* exposed by the
    compound sensor.

    :noautoexample:
    """
    _name = "Compound Sensor"
    _short_desc = "Special sensor that merges other sensors' outputs"

    # Set the values of image size from the variables
    #  in the Blender Logic Properties
    add_property('sensors', [], 'sensors', "The list of sensor names to merge.")

    def __init__(self, obj, parent=None):
        """ Constructor method.

        The object *must* have a property listing the sensors names to
        merge in this compound sensor. These sensors must be children (direct
        or not) of the parent of the compound sensor.

        Receives the reference to the Blender object.
        The second parameter is the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)

        if not self.sensors:
            logger.error("The compound sensor <%s> has no sensors list associated!" % obj.name)
            return

        self._initialized = False

    def _get_sensors_references(self):
        # Get the reference to all components
        component_dict = blenderapi.persistantstorage().componentDict

        for s in self.sensors.split(','):
            if s in component_dict:
                self.local_data[s] = component_dict[s].local_data
            else:
                logger.warning("Sensor <%s> not found while creating compound sensor <%s>!" % (s, self.name()))

        if self.local_data:
            logger.info('Compound sensor <%s> initialized with sensors %s' % (self.name(), list(self.local_data.keys())))
        else:
            logger.error("The compound sensor <%s> found none of the expected sensors!" % self.name())


    def default_action(self):
        """
        Default action does nothing at all, since self.local_data stores
        references to sub sensors' local_data, so it get updated for free.
        """
        if not self._initialized:
            self._get_sensors_references()
            self._initialized = True
