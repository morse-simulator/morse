from morse.middleware.ros_request_manager import ros_service
from morse.core.overlay import MorseOverlay
from morse.core import status

import roslib; roslib.load_manifest('morsetesting')
from morsetesting.srv import *

class WayPoint(MorseOverlay):

    def __init__(self, overlaid_object):
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(overlaid_object)
    
    @ros_service(type = MoveBase)
    def move_base(self, req):
        return self.overlaid_object.magic_goto(
                req.position.x,
                req.position.y,
                req.position.z)
