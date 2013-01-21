from morse.middleware.ros_request_manager import ros_service, ros_action
from morse.core.services import interruptible
from morse.core.overlay import MorseOverlay
from morse.core import status

import logging; logger = logging.getLogger("morse."+ __name__)

from pr2_controllers_msgs.msg import *

class ArmatureController(MorseOverlay):

    def __init__(self, overlaid_object):
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(overlaid_object)

    def _stamp_to_secs(self, stamp):
        return stamp.secs + stamp.nsecs / 1e9

    def joint_trajectory_action_result(self, result):
        return result

    @interruptible
    @ros_action(type = JointTrajectoryAction)
    def joint_trajectory_action(self, req):

        # Fill a MORSE trajectory structure from ROS JointTrajectory
        traj = {}

        req = req.trajectory
        traj["starttime"] = self._stamp_to_secs(req.header.stamp)

        points = []
        for p in req.points:
            point = {}
            point["positions"] = p.positions
            point["velocities"] = p.velocities
            point["accelerations"] = p.accelerations
            point["time_from_start"] = self._stamp_to_secs(p.time_from_start)
            points.append(point)

        traj["points"] = points
        logger.info(traj)
        
        self.overlaid_object.trajectory(
                self.chain_callback(self.joint_trajectory_action_result),
                traj)

