import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.datastream import DatastreamManager
from morse.core import blenderapi

from pymavlink.dialects.v10 import common as mavlink
from pymavlink.mavutil import mavlink_connection 

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
        datastream.setup(self._conn_manager, self._mav, self._boot_time)

    def action(self):
        self._conn_manager.send_hearbeat()
