import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.datastream import DatastreamManager
from morse.core import blenderapi

from pymavlink.dialects.v10 import common as mavlink
from pymavlink.mavutil import mavlink_connection 

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

class MavlinkConnManager:
    def __init__(self, mav):
        self._manager = {}
        self._mav = mav

    def get(self, conn):
        if conn not in self._manager:
            self._manager[conn] = mavlink_connection(conn)
        return self._manager[conn]


    def send_hearbeat(self):
        msg = mavlink.MAVLink_heartbeat_message(
                mavlink.MAV_TYPE_GENERIC, mavlink.MAV_AUTOPILOT_GENERIC,
                mavlink.MAV_MODE_TEST_ARMED, 0, mavlink.MAV_STATE_ACTIVE, 3)
        for conn in self._manager.values():
            conn.write(msg.pack(self._mav))

class MavlinkDatastreamManager(DatastreamManager):
    """ External communication using Mavlink protocol """

    def __init__(self, args, kwargs):
        """ Initialize the socket connections """
        # Call the constructor of the parent class
        DatastreamManager.__init__(self, args, kwargs)

        self._mav = mavlink.MAVLink(None)
        self._conn_manager = MavlinkConnManager(self._mav)
        self._boot_time = blenderapi.persistantstorage().time.time

    def register_component(self, component_name, component_instance, mw_data):
        """ Open the port used to communicate by the specified component.
        """

        # Create a socket server for this component
        datastream  = DatastreamManager.register_component(self, component_name,
                                         component_instance, mw_data)
        if hasattr(datastream, 'setup'):
            datastream.setup(self._conn_manager, self._mav, self._boot_time)

    def action(self):
        self._conn_manager.send_hearbeat()
