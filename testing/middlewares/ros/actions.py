#! /usr/bin/env python
"""
This script tests ROS actions within MORSE.
"""

from morse.testing.testing import MorseTestCase

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder.morsebuilder import *
except ImportError:
    pass

import os
import sys
try:
    os.environ['MORSE_SRC_ROOT']
except KeyError:
    print("You must define the environment variable MORSE_SRC_ROOT"
          " to point to the MORSE source before running ROS tests.")
    sys.exit(1)

os.environ['ROS_PACKAGE_PATH'] += ":" + os.path.dirname(
        os.path.join(os.environ['MORSE_SRC_ROOT'], "testing", "middlewares", "ros"))

import roslib; roslib.load_manifest("morsetesting")
import rospy
#import actionlib
import subprocess
from morsetesting.msg import *
from geometry_msgs.msg import *

class RosActionsTest(MorseTestCase):
    def setUpMw(self):
        try:
            self.roscore_process = subprocess.Popen(['roscore'])
        except OSError as ose:
            testlogger.error("Error while launching roscore ! Check you can run it from command-line\n")
            raise ose

    def tearDownMw(self):
        self.roscore_process.terminate()

    def setUpEnv(self):
        # Identical to ROS service testing
        
        print("Adding a robot...")
        robot = Robot('atrv')
        
        waypoint = Actuator('waypoint')
        robot.append(waypoint)
        
        waypoint.configure_service('ros')
        
        waypoint.configure_overlay('ros', 'morse.middleware.ros.overlays.actuator.WayPoint')
        
        env = Environment('indoors-1/indoor-1')
        env.configure_service('ros')

    def test_no_action(self):
            rospy.init_node('move_base_client')
            client = actionlib.SimpleActionClient('phantom_action', MoveBaseAction)
            self.assertFalse(client.wait_for_server(rospy.Duration(5)))

    def test_move_base(self):
            
            rospy.loginfo("Starting ROS test case for actions.")
            rospy.init_node('move_base_client')
            client = actionlib.SimpleActionClient('Motion_Controller/move_base', MoveBaseAction)
            self.assertTrue(client.wait_for_server(rospy.Duration(5)))

            goal = MoveBaseGoal(Pose(Point(0.1,3.0,0.0), Quaternion(0.0,0.0,0.0,1.0)))
            
            print("Sending a first goal to the robot...(timeout=5sec)")
            status = client.send_goal_and_wait(goal, rospy.Duration(5))

            print("Got this status: " + str(status))
            self.assertEqual(status, actionlib.GoalStatus.SUCCEEDED)



########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(RosActionsTest)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

