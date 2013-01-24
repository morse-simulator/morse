import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('asctec_msgs');
import math
from asctec_msgs.msg import CtrlInput
from morse.middleware.ros import ROSReader

class CtrlInputReader(ROSReader):

    def initialize(self):
        ROSReader.initialize(self, CtrlInput)

    def update(self, message):
        """ Method called as soon as CtrlInput messages are published on the specific topic """
        max_angle = math.radians(30)
        max_yaw_rate = math.radians(90)
        yaw_deadband = 5
        asctec_scale = 2047

        roll = message.roll / asctec_scale * max_angle
        pitch = message.pitch / asctec_scale * max_angle
        if math.fabs(message.yaw) > yaw_deadband:
            yaw_rate = max_yaw_rate / asctec_scale * (message.yaw - math.copysign(yaw_deadband, message.yaw))
        else:
            yaw_rate = 0.0
        thrust = message.thrust / 4095
        self.data["pitch"] = pitch
        self.data["roll"] = roll
        self.data["yaw"] = yaw_rate
        self.data["thrust"] = thrust

        logger.debug("new RPY thrust setpoint: (% .2f % .2f % .3f %3f)",
                     math.degrees(roll), math.degrees(pitch), math.degrees(yaw_rate), message.thrust)
