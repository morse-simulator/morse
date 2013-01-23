import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.datastream import *

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
