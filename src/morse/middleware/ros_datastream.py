import logging; logger = logging.getLogger("morse." + __name__)
import sys # sys.modules for get_class
from morse.core.sensor import Sensor
from morse.core.actuator import Actuator
from morse.core.datastream import Datastream
from morse.middleware import AbstractDatastream

def get_class(module_name, class_name):
    """ Dynamically import a Python class.
    """
    try:
        __import__(module_name)
    except ImportError as detail:
        logger.error("Module not found: %s" % detail)
        return None
    module = sys.modules[module_name]
    # Create an instance of the object class
    try:
        klass = getattr(module, class_name)
    except AttributeError as detail:
        logger.error("Module attribute not found: %s" % detail)
        return None
    return klass

def create_instance(classpath, *args, **kwargs):
    """ Creates an instances of a class.
    """
    module_name, class_name = classpath.rsplit('.', 1)
    klass = get_class(module_name, class_name)
    if klass:
        return klass(*args, **kwargs)
    else:
        logger.error("Could not create an instance of %s"%str(classpath))
    return None

def register_datastream(classpath, component, args):
    datastream = create_instance(classpath, component, args)
    # Check that datastream implements AbstractDatastream
    if not isinstance(datastream, AbstractDatastream):
        logger.error("%s must implements morse.middleware.AbstractDatastream"%classpath)
    # Check weither to store the function in input or output list
    # What is the direction of our stream?
    if isinstance(component, Sensor):
        # -> for Sensors, they *publish*,
        component.output_functions.append(datastream.default)
    elif isinstance(component, Actuator):
        # -> for Actuator, they *read*
        component.input_functions.append(datastream.default)
    else:
        logger.error("The component is not an instance of Sensor or Actuator")
        return
    # from morse.core.abstractobject.AbstractObject
    component.del_functions.append(datastream.finalize)


class ROS(Datastream):
    """ Handle communication between Blender and ROS."""

    def register_component(self, component_name, component_instance, mw_data):
        """ Generate a new instance of a datastream

        and register it to the component callbacks list
        """

        datastream_classpath = mw_data[1] # aka. function name
        # Check for new configuration
        if datastream_classpath.startswith('morse.middleware.ros.'):
            datastream_args = None
            if len(mw_data) > 2:
                datastream_args = mw_data[2] # aka. kwargs, a dictonnary of args
            register_datastream(datastream_classpath, component_instance, datastream_args)
        else:
            logger.error("%s must starts with 'morse.middleware.ros.'"%datastream_classpath)
