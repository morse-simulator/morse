import logging; logger = logging.getLogger("morse." + __name__)
# Modules necessary to dynamically add methods to Middleware subclasses

from morse.modifiers import AbstractModifier
from morse.core.sensor import Sensor
from morse.core.actuator import Actuator
from morse.helpers.loading import create_instance

def register_modifier(classpath, component, args):
    modifier = create_instance(classpath, component, args)
    if not modifier:
        logger.error("INITIALIZATION ERROR: Modifier '%s' module could not be "
                     "found!\n\n Could not import modules necessary for the "
                     "selected modifier. Check that they can be found inside "
                     "your PYTHONPATH variable." % classpath)
        return None

    # Check that modifier implements AbstractDatastream
    if not isinstance(modifier, AbstractModifier):
        logger.error("%s should implement morse.middleware.AbstractModifier" %
                     classpath)
        return None

    # Determine weither to store the function in input or output list,
    #   what is the direction of our stream?
    if isinstance(component, Sensor):
        # -> for Sensors, they *publish*,
        component.output_modifiers.append(modifier.modify)
    elif isinstance(component, Actuator):
        # -> for Actuator, they *read*
        component.input_modifiers.append(modifier.modify)
    else:
        logger.error("Component %s is not an instance of Sensor or Actuator" %
                     component.__class__)
        return None

    return modifier
