import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.mavlink.abstract_mavlink import MavlinkActuator
from pymavlink.quaternion import QuaternionBase

class AttitudeTarget(MavlinkActuator):
    _type_name = "SET_ATTITUDE_TARGET"

    def process_msg(self):
        q = QuaternionBase(self._msg.q)
        euler = q.euler()
        self.data['roll'] = euler[0]
        self.data['pitch'] = euler[1]
        self.data['roll'] = euler[2]
        self.data['thrust'] = self._msg.thrust
