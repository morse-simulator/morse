import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.mavlink.abstract_mavlink import MavlinkActuator

SCALE = 500
OFFSET = 340

class HILControls(MavlinkActuator):
    _type_name = "HIL_CONTROLS"

    def process_msg(self):
        self.data['engines'][1] = self._msg.roll_ailerons * SCALE + OFFSET
        self.data['engines'][3] = self._msg.pitch_elevator * SCALE + OFFSET
        self.data['engines'][0] = self._msg.yaw_rudder * SCALE + OFFSET
        self.data['engines'][2] = self._msg.throttle * SCALE + OFFSET

