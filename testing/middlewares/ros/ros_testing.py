#! /usr/bin/env python
"""
This script tests ROS services within MORSE.
"""

from morse.testing.testing import MorseTestCase

import os
os.environ['ROS_PACKAGE_PATH'] += os.path.dirname(
        os.path.join(os.environ['MORSE_ROOT'], "testing", "middlewares", "ros"))

import roslib; roslib.load_manifest("geometry_msgs"); roslib.load_manifest("morsetesting")
import rospy
from morsetesting.srv import *
from geometry_msgs.msg import *

class RosTest(MorseTestCase):

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
        
    def test_move_base(self):
        try:
            rospy.wait_for_service('move_base', timeout = 2)
        except rospy.ROSException:
            self.fail("The move_base service never showed up!")

        try:
            move_base = rospy.ServiceProxy('move_base', MoveBase)

            pose = Pose(Point(2.0,3.0,0.0), Quaternion(0.0,0.0,0.0,1.0))
            success = move_base(pose).success
            self.assertTrue(success)

            pose = Pose(Point(0.1,3.0,0.0), Quaternion(0.0,0.0,0.0,1.0))
            success = move_base(pose).success
            self.assertFalse(success)

        except rospy.ServiceException as e:
            self.fail("Service call failed: %s"%e)


########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(RosTest)
    MorseTestRunner().run(suite)

