""" The ROS 'datastream manager' is responsible for ROS topic management in
MORSE.  It publishes simulated sensors and subscribes to topics controlling
simulated actuators.

As you may have noticed, the
:py:class:`morse.middleware.ros_datastream.ROSDatastreamManager` class is
actually empty: contrary to sockets, for instance, that always use direct JSON
serialization of MORSE Python objects, there is no generic way to encode/decode
ROS messages.

Thus, `morse/middleware/ros` contains one specific
serialization/deserialization class for each sensor/actuator. These classes
inherit either from :py:class:`morse.middleware.ros.abstract_ros.ROSPublisher`
or from:py:class:`morse.middleware.ros.abstract_ros.ROSSubscriber`.

If you want to add support for a new type of topic, you likely want to add it
as a new serialization implementation in this directory.

Note also that management of ROS services and ROS actions takes place in
`ros_request_manager.py`.
"""
import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.datastream import DatastreamManager

class ROSDatastreamManager(DatastreamManager):
    """ Handle communication between Blender and ROS."""

