import logging; logger = logging.getLogger("morse." + __name__)
import datetime
from ctypes import *
from morse.middleware import AbstractDatastream
from morse.core.datastream import *

try:
    P = CDLL("libposterLib.so")
except OSError:
    import morse.core.blenderapi
    if not morse.core.blenderapi.fake:
        logger.error('Cannot find libposterLib.so : check your LD_LIBRARY_PATH')
        raise


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

class DummyPoster:
    def __init__(self, name):
        self.poster_id = c_void_p()
        c_name = create_string_buffer(bytes(name, 'utf-8'))
        P.posterCreate(c_name, 8, byref(self.poster_id))

    def __del__(self):
        P.posterDelete(self.poster_id)
        self.poster_id = None

class PocolibsDataStreamOutput(AbstractDatastream):

    def initialize(self, kind):
        self.poster_id = c_void_p()
        o = kind()
        self.name = poster_name(self.component_name, self.kwargs)
        c_name = create_string_buffer(bytes(self.name, 'utf-8'))
        logger.info("Create poster %s of size %d" % (self.name, sizeof(o)))
        r = P.posterCreate(c_name, sizeof(o), byref(self.poster_id))
        if r != 0:
            P.posterFind(c_name, byref(self.poster_id))

    def write(self, obj):
        r = P.posterWrite(self.poster_id, 0, byref(obj), sizeof(obj))
        if (r != sizeof(obj)):
            raise "too bad : write failed"

    def finalize(self):
        logger.info("Releaase poster %s" % (self.name))
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

    def compute_date():
        """ Compute the current time

        ( we only require that the date
        increases using a constant step so real time is ok)
        """
        t = datetime.datetime.now()
        date = int(t.hour * 3600* 1000 + t.minute * 60 * 1000 +
                t.second * 1000 + t.microsecond / 1000)

        return date, t
