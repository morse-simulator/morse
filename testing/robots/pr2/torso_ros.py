#! /usr/bin/env python
"""
This script tests the PR2 torso armature joint
"""

from morse.testing.ros import RosTestCase

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

import sys
import time

import rospy
from control_msgs.msg import SingleJointPositionAction, SingleJointPositionGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import actionlib

def getjoint(name):

    data = rospy.wait_for_message("/pr2/joint_states", JointState)
    idx = data.name.index(name)
    return data.position[idx]

class PR2TorsoTest(RosTestCase):

    def setUpEnv(self):
        print("Adding a PR2 robot...")
        pr2 = BasePR2()
        pr2.add_interface('ros')

        env = Environment('empty', fastmode=True)
        env.set_camera_rotation([1.0470, 0, 0.7854])

    def test_controller(self):

        topic = "/pr2/torso_controller/command"

        rospy.init_node('test_pr2_torso', disable_signals=True)

        rospy.loginfo("Preparing to publish on %s" % topic)
        ctl = rospy.Publisher(topic, JointTrajectory)

        self.assertEquals(getjoint("torso_lift_joint"), 0.0)

        duration = 0.1

        traj = JointTrajectory()
        traj.joint_name = "torso_lift_joint"

        initialpoint = JointTrajectoryPoint()
        initialpoint.positions = 0.0
        initialpoint.velocities = 0.0
        initialpoint.time_from_start = rospy.Duration(0.0)

        finalpoint = JointTrajectoryPoint()
        finalpoint.positions = 0.195
        finalpoint.velocities = 0.0
        finalpoint.time_from_start = rospy.Duration(duration)

        # First, move up 
        traj.points = [initialpoint, finalpoint]

        ctl.publish(traj)
        time.sleep(duration + 0.1)
        self.assertEquals(getjoint("torso_lift_joint"), 0.195)

        # Go back to initial position
        finalpoint.time_from_start = rospy.Duration(0.0)
        initialpoint.time_from_start = rospy.Duration(duration)
        traj.points = [finalpoint, initialpoint]

        ctl.publish(traj)
        time.sleep(duration + 0.1)
        self.assertEquals(getjoint("torso_lift_joint"), 0.0)

    def test_action(self):


        rospy.loginfo("Trying to move PR2 torso at action level.")
        rospy.init_node('test_pr2_torso', disable_signals=True)
        client = actionlib.SimpleActionClient('torso_controller/position_joint_action', SingleJointPositionAction)
        self.assertTrue(client.wait_for_server(rospy.Duration(5)))

        self.assertEquals(getjoint("torso_lift_joint"), 0.0)

        up = SingleJointPositionGoal
        up.position = 0.195
        up.min_duration = rospy.Duration(2.0)
        up.max_velocity = 1.0

        print("Sending a 'up' goal to the torso...(timeout=5sec)")
        status = client.send_goal_and_wait(up, rospy.Duration(5))

        print("Got this status: " + str(status))
        self.assertEqual(status, actionlib.GoalStatus.SUCCEEDED)

        self.assertEquals(getjoint("torso_lift_joint"), 0.195)

        down = SingleJointPositionGoal
        down.position = 0.0
        down.min_duration = rospy.Duration(2.0)
        down.max_velocity = 1.0

        print("Sending a 'down' goal to the torso...(timeout=5sec)")
        status = client.send_goal_and_wait(down, rospy.Duration(5))

        print("Got this status: " + str(status))
        self.assertEqual(status, actionlib.GoalStatus.SUCCEEDED)

        self.assertEquals(getjoint("torso_lift_joint"), 0.0)


########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(PR2TorsoTest)
