import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.datastream import DatastreamManager


class PprzlinkDatastreamManager(DatastreamManager):
    """ External communication using PPRZLINK protocol over IVY """

