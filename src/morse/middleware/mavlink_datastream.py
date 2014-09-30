import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.datastream import DatastreamManager
from morse.core import blenderapi

from pymavlink.dialects.v10 import common as mavlink

import socket


class MavlinkClient:
    def __init__(self, ip, input_port, output_port):
        self._udp = socket.socket(socket.AF_INET,  socket.SOCK_DGRAM)
        self._udp.bind((ip, input_port))
        self._out_addr = (ip, output_port)
        self._mav = mavlink.MAVLink(None)

    def send(self, msg):
        err = self._udp.sendto(msg.pack(self._mav), self._out_addr)
        logger.debug("Sending %s (%d bytes)" % (msg, err))

    def __finalize__(self):
        self._udp.close()

class MavlinkDatastreamManager(DatastreamManager):
    """ External communication using Mavlink protocol """

    def __init__(self, args, kwargs):
        """ Initialize the socket connections """
        # Call the constructor of the parent class
        DatastreamManager.__init__(self, args, kwargs)

        # Create one client for each robot, as 'expected by mavlink
        # protocol'
        self._client_by_robot = {}

        self._boot_time = blenderapi.persistantstorage().time.time

    def register_component(self, component_name, component_instance, mw_data):
        """ Open the port used to communicate by the specified component.
        """
        # Create a socket server for this component
        datastream  = DatastreamManager.register_component(self, component_name,
                                         component_instance, mw_data)
        
        client = self._client_by_robot.get(component_instance.robot_parent.name, None)
        if not client:
            client = MavlinkClient('127.0.0.1', 14551, 14550) #XXX 
            self._client_by_robot[component_instance.robot_parent.name] = client

        datastream.setup(client, self._boot_time)

    def action(self):
        for client in self._client_by_robot.values():
            msg = mavlink.MAVLink_heartbeat_message(
                    mavlink.MAV_TYPE_GENERIC, mavlink.MAV_AUTOPILOT_GENERIC,
                    mavlink.MAV_MODE_TEST_ARMED, 0, mavlink.MAV_STATE_ACTIVE, 3)
            client.send(msg)
