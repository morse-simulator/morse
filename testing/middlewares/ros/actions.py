#! /usr/bin/env python
"""
This script tests ROS actions within MORSE.
"""

from morse.testing.ros import RosTestCase

# Include this import to be able to use your test file as a regular
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

import os
import sys
import time

import rospy
import actionlib
import nav_msgs.msg # do not conflict with morse builder

from move_base_msgs.msg import *

from geometry_msgs.msg import *

class RosActionsTest(RosTestCase):

    def setUpEnv(self):
        # Identical to ROS service testing

        print("Adding a robot...")
        robot = ATRV()

        odometry = Odometry()
        robot.append(odometry)
        odometry.add_stream('ros')

        waypoint = Waypoint()
        robot.append(waypoint)

        waypoint.add_service('ros')

        waypoint.add_overlay('ros', 'morse.middleware.ros.overlays.waypoints.WayPoint')

        env = Environment('empty', fastmode = True)
        env.add_service('ros')

    def test_no_action(self):
            rospy.init_node('move_base_client')
            client = actionlib.SimpleActionClient('phantom_action', MoveBaseAction)
            self.assertFalse(client.wait_for_server(rospy.Duration(5)))

    def test_move_base(self):

            rospy.loginfo("Starting ROS test case for actions.")
            rospy.init_node('move_base_client')
            client = actionlib.SimpleActionClient('robot/waypoint/move_base', MoveBaseAction)
            self.assertTrue(client.wait_for_server(rospy.Duration(5)))

            goal = MoveBaseGoal(PoseStamped(pose = Pose(Point(0.1,3.0,0.0), Quaternion(0.0,0.0,0.0,1.0))))

            print("Sending a first goal to the robot...(timeout=5sec)")
            status = client.send_goal_and_wait(goal, rospy.Duration(5))

            print("Got this status: " + str(status))
            self.assertEqual(status, actionlib.GoalStatus.SUCCEEDED)

    def get_pose(self):
        msg = rospy.client.wait_for_message('/robot/odometry', nav_msgs.msg.Odometry, timeout = 10)
        return (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def check_not_moving(self):
        pos1 = self.get_pose()
        time.sleep(1.0)
        pos2 = self.get_pose()
        self.assertAlmostEqual(pos1[0], pos2[0], delta=0.15)
        self.assertAlmostEqual(pos1[1], pos2[1], delta=0.15)

    def cb_preempted(self, status, res):
        self.cb_fired = True
        self.assertEqual(status, actionlib.GoalStatus.PREEMPTED)

    def cb_succeeded(self, status, res):
        self.cb_fired = True
        self.assertEqual(status, actionlib.GoalStatus.SUCCEEDED)


    def test_move_advanced(self):

        rospy.loginfo("Starting ROS test case for actions (advanced behaviour).")
        rospy.init_node('move_base_client')
        client = actionlib.SimpleActionClient('robot/waypoint/move_base', MoveBaseAction)
        client2 = actionlib.SimpleActionClient('robot/waypoint/move_base', MoveBaseAction)
        self.assertTrue(client.wait_for_server(rospy.Duration(5)))
        self.assertTrue(client2.wait_for_server(rospy.Duration(5)))

        self.assertIsNotNone(self.get_pose())

        goal1 = MoveBaseGoal(PoseStamped(pose = Pose(Point(0.1,10.0,0.0), Quaternion(0.0,0.0,0.0,1.0))))
        goal2 = MoveBaseGoal(PoseStamped(pose = Pose(Point(0.1,2.0,0.0), Quaternion(0.0,0.0,0.0,1.0))))


        # send goal with not enough time to complete -> should get a preemption and the robot actually stopping
        status = client.send_goal_and_wait(goal1, rospy.Duration(1))
        self.assertEqual(status, actionlib.GoalStatus.PREEMPTED)
        self.check_not_moving()
        ######

        # sending again the goal, and ask for cancellation
        self.cb_fired = False
        client.send_goal(goal1, done_cb = self.cb_preempted)
        time.sleep(1)
        client.cancel_goal()
        self.check_not_moving()
        self.assertTrue(self.cb_fired)
        ######

        # sending again the goal, this time, interrupted by another one
        self.cb_fired = False
        client.send_goal(goal1, done_cb = self.cb_preempted)
        time.sleep(1)
        client2.send_goal(goal2, done_cb = self.cb_succeeded)
        time.sleep(0.5)
        self.assertTrue(self.cb_fired)
        self.cb_fired = False
        time.sleep(5)
        self.assertTrue(self.cb_fired)
        ######




########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(RosActionsTest, time_modes = [TimeStrategies.BestEffort])
