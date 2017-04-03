import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.pprzlink.abstract_pprzlink import PprzlinkActuator
from pprzlink.message import PprzMessage 

def pos_of_int(pos):
    return float(pos) / 2**8

def angle_of_int(angle):
    return float(angle) / 2**12

""" Set rotorcraft pose using the Teleport actuator """
class RotorcraftPose(PprzlinkActuator):
    _type_name = "ROTORCRAFT_FP"

    def process_msg(self):
        # the actuator assumes ned control, so don't do any transformation
        self.data['x'] = pos_of_int(self._msg['east'])
        self.data['y'] = pos_of_int(self._msg['north'])
        self.data['z'] = pos_of_int(self._msg['up'])
        self.data['roll']   = angle_of_int(self._msg['phi'])
        self.data['pitch']  = angle_of_int(self._msg['theta'])
        self.data['yaw']    = angle_of_int(self._msg['psi'])

