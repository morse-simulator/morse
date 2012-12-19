import logging; logger = logging.getLogger("morse." + __name__)
import datetime
from ctypes import *

import morse.core.datastream

from morse.middleware.pocolibs.sensors.General_Poster import ors_poster

P = CDLL("libposterLib.so")

class PosterNotFound(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class InvalidRead(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

def poster_name(component, mw_data):
    # Check if the name of the poster has been given in mw_data
    poster_name = ''
    try:
        # It should be the 4th parameter
        poster_name = mw_data[3]
    except IndexError:
        # Compose the name of the poster, based on the parent and module names
        component_name = component.bge_obj.name
        parent_name = component.robot_parent.bge_obj.name
        poster_name = '{0}_{1}'.format(parent_name, component_name)

    return poster_name

class PocolibsDataStreamOutput(object):

    def __init__(self, name, kind):
        self.poster_id = c_void_p()
        o = kind()
        c_name = create_string_buffer(bytes(name, 'utf-8'))
        logger.info("Create poster %s of size %d\n" % (name, sizeof(o)))
        r = P.posterCreate(c_name, sizeof(o), byref(self.poster_id))
        if r != 0:
            P.posterFind(c_name, byref(self.poster_id))

    def write(self, obj):
        r = P.posterWrite(self.poster_id, 0, byref(obj), sizeof(obj))
        if (r != sizeof(obj)):
            raise "too bad : write failed"

    def __del__(self):
        P.posterDelete(self.poster_id)
        self.poster_id = None

class PocolibsDataStreamInput(object):
    def __init__(self, name, delay, kind):
        self.poster_id = c_void_p()
        self.name = name
        self.c_name = create_string_buffer(bytes(name, 'utf-8'))

        self.o = kind()
        self.found = False
        self._find()
        if (not self.found and not delay):
            raise PosterNotFound(self.name)

    def _find(self):
        logger.info("Searching to read %s" % self.name)
        r = P.posterFind(self.c_name, byref(self.poster_id))
        if (r == 0):
            self.found = True

    def read(self):
        if (not self.found):
            self._find()

        if (self.found):
            r = P.posterRead(self.poster_id, 0, byref(self.o), sizeof(self.o))
            if (r != sizeof(self.o)):
                raise InvalidRead(self.name)
            return self.o
        else:
            return None

    def __del__(self):
        if (self.found):
            P.posterForget(self.poster_id)


class Pocolibs(morse.core.datastream.Datastream):
    """ Handle communication between Blender and Pocolibs."""

    def __init__(self):
        # Call the constructor of the parent class
        super(self.__class__,self).__init__()

        # Store the id's of created posters, indexed by component name
        self._poster_dict = dict()
        # Dictionary for external posters
        self._poster_in_dict = dict()
        self._imported_modules = dict()


    def __del__(self):
        """ Close all open posters. """
        for component_name, poster_id in self._poster_dict.items():
            logger.info("Killing poster %d for component %s" % (poster_id, component_name))
            # Call the method to close a poster
            ors_poster.finalize(poster_id)


    def register_component(self, component_name, component_instance, mw_data):
        """ Open the pocolibs poster to communicate a specified component.

        The configuration of each type of poster is done in external modules.
        """
        # Import the configuration of the poster from an extra module
        function = self._add_method(mw_data, component_instance)


    def _compute_date(self):
        """ Compute the current time

        ( we only require that the date
        increases using a constant step so real time is ok)
        """
        t = datetime.datetime.now()
        date = int(t.hour * 3600* 1000 + t.minute * 60 * 1000 +
                t.second * 1000 + t.microsecond / 1000)

        return date, t


def init_extra_actuator(self, component_instance, function, mw_data, kind):
    """ Setup the middleware connection with this data
    Prepare the middleware to handle the serialised data as necessary.
    """
    name = poster_name(component_instance, mw_data)

    logger.debug("Creating poster_name %s" % name)
    poster_id = kind.createPosterHandler(name)
    self._poster_in_dict[component_instance.blender_obj.name] = poster_id
    component_instance.input_functions.append(function)
