import logging; logger = logging.getLogger("morse." + __name__)

from morse.core.datastream import Datastream

class MOOS(Datastream):
    """ Handle communication between Blender and MOOS."""
