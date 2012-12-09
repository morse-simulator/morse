import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('asctec_msgs');
import math
from asctec_msgs.msg import CtrlInput

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    self.register_subscriber(component_instance, function, CtrlInput, callback_ctrl_input)

def callback_ctrl_input(data, component_instance):
    """ this function is called as soon as asctec CtrlInput messages are published on the specific topic """

    max_angle = math.radians(30)
    max_yaw_rate = math.radians(90)
    yaw_deadband = 5
    asctec_scale = 2047

    roll = data.roll / asctec_scale * max_angle
    pitch = data.pitch / asctec_scale * max_angle
    if math.fabs(data.yaw) > yaw_deadband:
        yaw_rate = max_yaw_rate / asctec_scale * (data.yaw - math.copysign(yaw_deadband, data.yaw))
    else:
        yaw_rate = 0.0
    thrust = data.thrust / 4095
    component_instance.local_data["pitch"] = pitch
    component_instance.local_data["roll"] = roll
    component_instance.local_data["yaw"] = yaw_rate
    component_instance.local_data["thrust"] = thrust

    logger.debug("new RPY thrust setpoint: (% .2f % .2f % .3f %3f)",
                 math.degrees(roll), math.degrees(pitch), math.degrees(yaw_rate), data.thrust)

def read_ctrl_input(self, component_instance):
    """ dummy function Asctec CtrlInput """
