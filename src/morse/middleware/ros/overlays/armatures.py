from morse.middleware.ros_request_manager import ros_action
from morse.core.services import interruptible
from morse.core.overlay import MorseOverlay
from morse.core import status
from morse.core.exceptions import MorseServiceError

import logging; logger = logging.getLogger("morse."+ __name__)

import rospy # to set the parameters

# pr2_controllers_msgs is not catkinized in fuerte
from morse.middleware.ros.helpers import ros_add_to_syspath
ros_add_to_syspath("pr2_controllers_msgs")

from pr2_controllers_msgs.msg import *

class ArmatureController(MorseOverlay):
    """
    This overlay provides a ROS JointTrajectoryAction interface to armatures.

    It is meant to be applied on a Armature actuator.

    Besides the ROS action server, it also sets a ROS parameter with the list of
    joints.
    """

    def __init__(self, overlaid_object, namespace = None):
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(overlaid_object)

        joints = list(overlaid_object.local_data.keys())

        self.namespace = namespace

        name = self.name().replace(".", "/")

        rospy.set_param(name + "/joints", joints)
        rospy.set_param(name + "/joint_trajectory_action/joints", joints)

    def _stamp_to_secs(self, stamp):
        return stamp.secs + stamp.nsecs / 1e9

    def joint_trajectory_action_result(self, result):
        return result

    def name(self):

        if self.namespace:
            return self.namespace
        else:
            return super(self.__class__, self).name()

    @interruptible
    @ros_action(type = JointTrajectoryAction)
    def joint_trajectory_action(self, req):

        # Fill a MORSE trajectory structure from ROS JointTrajectory
        traj = {}

        req = req.trajectory
        traj["starttime"] = self._stamp_to_secs(req.header.stamp)

        joint_names = req.joint_names
        target_joints = self.overlaid_object.local_data.keys()
        diff = set(joint_names).difference(set(target_joints))

        if diff:
            raise MorseServiceError("Trajectory contains unknown joints! %s" % diff)

        points = []
        for p in req.points:
            point = {}

            # Re-order joint values to match the local_data order

            pos = dict(zip(joint_names, p.positions))
            point["positions"] = [pos[j] for j in target_joints if j in pos]
            vel = dict(zip(joint_names, p.velocities))
            point["velocities"] = [vel[j] for j in target_joints if j in vel]

            acc = dict(zip(joint_names, p.accelerations))
            point["accelerations"] = [acc[j] for j in target_joints if j in acc]

            point["time_from_start"] = self._stamp_to_secs(p.time_from_start)
            points.append(point)

        traj["points"] = points
        logger.info(traj)
        
        self.overlaid_object.trajectory(
                self.chain_callback(self.joint_trajectory_action_result),
                traj)

