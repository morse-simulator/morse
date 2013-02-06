import logging; logger = logging.getLogger("morse." + __name__)
import datetime
from ctypes import *
from morse.middleware import AbstractDatastream
from morse.core.datastream import *

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

def poster_name(component_name, mw_data):
    # Check if the name of the poster has been given in mw_data
    poster_name = ''
    if 'poster' in mw_data:
        poster_name = mw_data['poster']
    else:
        # Compose the name of the poster, based on the parent and module names
        poster_name = component_name

    return poster_name

class PocolibsDataStreamOutput(AbstractDatastream):

    def initialize(self, kind):
        self.poster_id = c_void_p()
        o = kind()
        name = poster_name(self.component_name, self.kwargs)
        c_name = create_string_buffer(bytes(name, 'utf-8'))
        logger.info("Create poster %s of size %d\n" % (name, sizeof(o)))
        r = P.posterCreate(c_name, sizeof(o), byref(self.poster_id))
        if r != 0:
            P.posterFind(c_name, byref(self.poster_id))

    def write(self, obj):
        r = P.posterWrite(self.poster_id, 0, byref(obj), sizeof(obj))
        if (r != sizeof(obj)):
            raise "too bad : write failed"

    def finalize(self):
        P.posterDelete(self.poster_id)
        self.poster_id = None

class PocolibsDataStreamInput(AbstractDatastream):
    def initialize(self, kind):
        self.poster_id = c_void_p()
        name = poster_name(self.component_name, self.kwargs)
        delay = self.kwargs.get('delay', True)
        self.name = name
        self.c_name = create_string_buffer(bytes(name, 'utf-8'))

        self.o = kind()
        self.found = False
        self._find()
        if (not self.found and not delay):
            raise PosterNotFound(self.name)

    def _find(self):
        logger.debug("Searching to read %s" % self.name)
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

    def finalize(self):
        if (self.found):
            P.posterForget(self.poster_id)

class Pocolibs(Datastream):
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


    def register_component(self, component_name, component_instance, mw_data):
        """ Open the port used to communicate by the specified component.
        """

        datastream_classpath = mw_data[1] # aka. function name
        datastream_args = None
        if len(mw_data) > 2:
            datastream_args = mw_data[2] # aka. kwargs, a dictonnary of args

        # Create a socket server for this component
        register_datastream(datastream_classpath, component_instance, datastream_args)



    def compute_date():
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
