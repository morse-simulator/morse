#! /usr/bin/env python
"""
This script tests ROS services within MORSE.
"""

from morse.testing.testing import MorseTestCase

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
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
import subprocess
from morsetesting.srv import *
from morsetesting.msg import *
from geometry_msgs.msg import *

class RosServicesTest(MorseTestCase):
    def setUpMw(self):
        try:
            self.roscore_process = subprocess.Popen(['roscore'])
        except OSError as ose:
            testlogger.error("Error while launching roscore ! Check you can run it from command-line\n")
            raise ose

    def tearDownMw(self):
        self.roscore_process.terminate()

    def setUpEnv(self):
        
        print("Adding a robot...")
        robot = Robot('atrv')
        
        waypoint = Actuator('waypoint')
        robot.append(waypoint)
        
        waypoint.configure_service('ros')
        
        waypoint.configure_overlay('ros', 'morse.middleware.ros.overlays.actuator.WayPoint')
        
        env = Environment('indoors-1/indoor-1')
        env.configure_service('ros')

    def test_unknow_service(self):
        
        with self.assertRaises(rospy.ROSException):
            rospy.wait_for_service('idonotexist', timeout = 2)
        
    def test_set_destination(self):

        try:
            rospy.wait_for_service('Motion_Controller/set_destination', timeout = 2)
        except rospy.ROSException:
            self.fail("The set_destination service never showed up!")

        try:
            set_dest = rospy.ServiceProxy('Motion_Controller/set_destination', MoveBase)

            
            # Send a destination target at the robot current position ->
            # should return False
            pose = Pose(Point(0.0,0.0,0.0), Quaternion(0.0,0.0,0.0,1.0))
            success = set_dest(pose).success
            self.assertFalse(success)

            # Send a destination target within tolerance (default = 0.5) of robot current position ->
            # should return False
            pose = Pose(Point(0.1,0.3,0.0), Quaternion(0.0,0.0,0.0,1.0))
            success = set_dest(pose).success
            self.assertFalse(success)

            # Send a new destination target
            pose = Pose(Point(-1.0,3.0,0.0), Quaternion(0.0,0.0,0.0,1.0))
            success = set_dest(pose).success
            self.assertTrue(success)

            # Override the previous target
            pose = Pose(Point(1.0,0.0,0.0), Quaternion(0.0,0.0,0.0,1.0))
            success = set_dest(pose).success
            self.assertTrue(success)

        except rospy.ServiceException as e:
            self.fail("Service call failed: %s"%e)

########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(RosServicesTest)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

