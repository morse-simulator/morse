import logging; logger = logging.getLogger("morse." + __name__)
import math
from sys import maxsize
from morse.core import blenderapi
import morse.core.sensor
from morse.helpers.components import add_data, add_property

class Thermometer(morse.core.sensor.Sensor):
    """
    This sensor emulates a Thermometer, measuring the temperature with respect
    to the distance to heat sources. It defines a default temperature throughout
    the scenario, which is affected by local fire sources. The temperature rises
    exponentially when the distance between the sensor and the heat source
    decreases. Its equation is given by:
    
    .. math::

     temperature = DefaultTemperature + \\Sigma_{s} FireTemperature(s) * \\exp ( - \\alpha * distance(s) )
     
    Each fire source must define a property named as the FireTag (default is 'Fire').
    If this property is an int or a float, its value is used as the source fire temperature. 
    """

    _name = "Thermometer Sensor"

    add_property('_tag', "Fire", "FireTag", "string",
                 "Tag indicating that an object is a fire source")
    add_property('_zero', 15.0, "DefaultTemperature", "float", 
                 "Default temperature: returned by the sensor when no source is detected")
    add_property('_fire', 200.0, "FireTemperature", "float",
                 "Temperature of fire sources. Can be overriden by objects using the FireTag property")
    add_property('_range', maxsize, "Range", "float",
                 "Maximum distance to which fire sources are detected")
    add_property('_alpha', .2, "Alpha", "float",
                 "Attenuation coefficient alpha")

    add_data('temperature', 0.0, "float", "Temperature in Celsius")

    def __init__(self, obj, parent=None):
        """ Constructor method.
            Receives the reference to the Blender object.
            The second parameter should be the name of the object's parent. """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)

        logger.info("Component initialized, runs at %.2f Hz ", self.frequency)

    def default_action(self):
        """ Compute the local temperature

        Temperature is measured dependent on the closest fire sources.
        """
        
        temp = float(self._zero)

        scene = blenderapi.scene()
        # Look for the fire sources marked so
        for obj in scene.objects:
            try:
                f = obj[self._tag]
                if type(f) == int or type(f) == float:
                    fire_intensity = float(f)
                else:
                    fire_intensity = self._fire
                    
                distance, gvect, lvect = self.bge_object.getVectTo(obj)
                if distance < self._range:
                    t = fire_intensity * math.exp(- self._alpha * distance)
                    temp += t
                    
            except KeyError as detail:
                logger.debug("Exception: " + str(detail))

        self.local_data['temperature'] = float(temp)
