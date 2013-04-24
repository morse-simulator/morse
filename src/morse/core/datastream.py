import logging; logger = logging.getLogger("morse." + __name__)
# Modules necessary to dynamically add methods to Middleware subclasses
import os
import sys
import re
import types

from abc import ABCMeta, abstractmethod

from morse.core.sensor import Sensor
from morse.core.actuator import Actuator
from morse.middleware import AbstractDatastream
from morse.helpers.loading import create_instance


def register_datastream(classpath, component, args):
    datastream = create_instance(classpath, component, args)
    # Check that datastream implements AbstractDatastream
    if not isinstance(datastream, AbstractDatastream):
        logger.warning("%s should implement morse.middleware.AbstractDatastream"%classpath)
    # Determine weither to store the function in input or output list,
    #   what is the direction of our stream?
    if isinstance(component, Sensor):
        # -> for Sensors, they *publish*,
        component.output_functions.append(datastream.default)
    elif isinstance(component, Actuator):
        # -> for Actuator, they *read*
        component.input_functions.append(datastream.default)
    else:
        logger.error("The component is not an instance of Sensor or Actuator")
        return None
    # from morse.core.abstractobject.AbstractObject
    component.del_functions.append(datastream.finalize)

    return datastream


class Datastream(object):
    """ Basic Class for all middlewares

    Provides common attributes. """

    # Make this an abstract class
    __metaclass__ = ABCMeta

    def __del__(self):
        """ Destructor method. """
        logger.info("Closing datastream interface <%s>." % self.__class__.__name__)


    def register_component(self, component_name, component_instance, mw_data):
        datastream_classpath = mw_data[1] # aka. function name
        datastream_args = None
        if len(mw_data) > 2:
            datastream_args = mw_data[2] # aka. kwargs, a dictonnary of args

        # Create a socket server for this component
        return register_datastream(datastream_classpath, component_instance,
                                                         datastream_args)


