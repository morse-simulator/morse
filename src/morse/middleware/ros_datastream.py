import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.datastream import Datastream

class ROS(Datastream):
    """ Handle communication between Blender and ROS."""

