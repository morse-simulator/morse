from morse.middleware.ros_request_manager import ros_service
from morse.core.overlay import MorseOverlay
from morse.core import status

import roslib; roslib.load_manifest('morsetesting')
from morsetesting.srv import *

class PR2(MorseOverlay):

    def __init__(self, overlaid_object):
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(overlaid_object)
    
    @ros_service(type = GetHead)
    def get_head(self):
        #print("Head rotations: %s"%self.overlaid_object.get_rotations())
        return self.overlaid_object.get_rotations()w
    
    @ros_service(type = SetHead)
    def set_head(self, pan, tilt):
        #print("Setting head to: %s"%pan, tilt)
        self.overlaid_object.set_rotation("head_pan", [0, pan, 0])
        self.overlaid_object.set_rotation("head_tilt", [0, 0, tilt])
        return True
        
