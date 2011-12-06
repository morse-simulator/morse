from morse.middleware.ros_request_manager import ros_action, ros_service
from morse.core.overlay import MorseOverlay
from morse.core import status

import roslib; roslib.load_manifest('morsetesting')
from morsetesting.msg import *
from morsetesting.srv import *

class WayPoint(MorseOverlay):

    def __init__(self, overlaid_object):
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(overlaid_object)

    def move_base_on_completion(self, result):
        state, value = result

        print("MoveBase completed!! got value " + str(value))

        if state == status.SUCCESS:
            res = MoveBaseResult(success = True)
        else:
            res = None

        return (state, res)

    @ros_action(type = MoveBaseAction)
    def move_base(self, req):
        self.overlaid_object.goto(
                self.chain_callback(self.move_base_on_completion),
                req.target.position.x,
                req.target.position.y,
                req.target.position.z)

    @ros_service(type = MoveBase)
    def set_destination(self, req):
        return self.overlaid_object.setdest(
                            req.position.x,
                            req.position.y,
                            req.position.z)

