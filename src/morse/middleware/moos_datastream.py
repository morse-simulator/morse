import logging; logger = logging.getLogger("morse." + __name__)

from morse.core.datastream import DatastreamManager

class MOOSDatastreamManager(DatastreamManager):
    """ Handle communication between Blender and MOOS."""

