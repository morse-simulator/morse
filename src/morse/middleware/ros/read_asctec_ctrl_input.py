import logging; logger = logging.getLogger("morse." + __name__)
import math
from asctec_msgs.msg import CtrlInput
from morse.middleware.ros import ROSSubscriber

class CtrlInputReader(ROSSubscriber):
    """ Subscribe to a CtrlInput topic and set pitch,roll,yaw,thrust local data. """
    ros_class = CtrlInput

    def update(self, message):
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
