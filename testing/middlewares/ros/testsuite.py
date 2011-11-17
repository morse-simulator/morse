#! /usr/bin/env python

import sys, os
import subprocess
import unittest

def start_simulator():
        try:
            subprocess.Popen(
                  ['morse', 'run', __file__])
        except (OSError, IndexError):
            return None


"""
This script tests ROS services within MORSE.
"""

def env_setup():
    
    robot = Robot('atrv')
    
    waypoint = Actuator('waypoint')
    robot.append(waypoint)
    
    waypoint.configure_service('ros')
    
    waypoint.configure_overlay('ros', 'morse.middleware.ros.overlays.actuator.WayPoint')
    
    ######### ENVIRONEMENT ########
    
    env = Environment('indoors-1/indoor-1')
    env.configure_service('ros')

class RosTest(unittest.TestCase):

    def setUp(self):
        pass

    def test_list_robots(self):
        try:
            rospy.wait_for_service('list_robots', timeout = 2)
        except rospy.ROSException:
            self.fail("The list_robots service never showed up!")
        try:
            list_robots = rospy.ServiceProxy('list_robots', ListRobots)

            robots = list_robots()

            self.assertIs(robots, ['ATRV'])

        except rospy.ServiceException as e:
            self.fail("Service call failed: %s"%e)

    def test_move_base(self):
        try:
            rospy.wait_for_service('move_base', timeout = 2)
        except rospy.ROSException:
            self.fail("The move_base service never showed up!")

        try:
            move_base = rospy.ServiceProxy('move_base', MoveBase)

            pose = Pose(Point(2.0,3.0,0.0), Quaternion(0.0,0.0,0.0,1.0))
            success = move_base(pose)
            self.assertTrue(success)

            pose = Pose(Point(0.1,3.0,0.0), Quaternion(0.0,0.0,0.0,1.0))
            success = move_base(pose)
            self.assertFalse(success)

        except rospy.ServiceException as e:
            self.fail("Service call failed: %s"%e)

def start_roscore():
        try:
            subprocess.Popen(['roscore'])
        except (OSError, IndexError):
            return None

##### Executed code starts here ######
if sys.argv[0].endswith('blender'): # Running inside Blender -> Call MORSE Builder directly!
    from morse.builder.morsebuilder import *
    env_setup()
else:
    os.environ['ROS_PACKAGE_PATH'] += os.path.dirname(__file__)
    import roslib; roslib.load_manifest("geometry_msgs"); roslib.load_manifest("morsetesting")
    import rospy
    from morsetesting.srv import *
    from geometry_msgs.msg import *

    start_roscore()
    start_simulator()

    print("Waiting 10 sec for the simulator to be ready...")
    import time
    time.sleep(10)

    unittest.main()


