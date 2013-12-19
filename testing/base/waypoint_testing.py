#! /usr/bin/env python
"""
This script tests the waypoints actuator, both the data and service api
"""
import logging

import sys
import math
from morse.testing.testing import MorseTestCase
from pymorse import Morse, MorseServicePreempted

logger = logging.getLogger("morsetesting.general")
# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

class Waypoints_Test(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        
        robot = ATRV("robot")

        pose = Pose('pose')
        pose.translate(z=-0.10) # atrv body
        robot.append(pose)
        pose.add_stream('socket')

        motion = Waypoint('motion')
        robot.append(motion)
        motion.add_stream('socket')
        motion.add_service('socket')

        
        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def test_waypoint_datastream(self):
        """ This test is guaranteed to be started only when the simulator
        is ready.
        """
        with Morse() as simu:
        
            # Read the start position, it must be (0.0, 0.0, 0.0)
            pose_stream = simu.robot.pose
            pose = pose_stream.get()
            for key, coord in pose.items():
                if key != 'timestamp':
                    self.assertAlmostEqual(coord, 0.0, delta=0.02)

            # waypoint controller
            motion = simu.robot.motion
            motion.publish({'x' : 4.0, 'y': 2.0, 'z': 0.0, 
                            'tolerance' : 0.5, 
                            'speed' : 1.0})
            simu.sleep(10)

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 4.0, delta=0.5)
            self.assertAlmostEqual(pose['y'], 2.0, delta=0.5)


            # test tolerance parameter
            motion.publish({'x' : 0.0, 'y': 0.0, 'z': 0.0, 
                            'tolerance' : 1.0, 
                            'speed' : 1.0})
            simu.sleep(10)
            pose = pose_stream.get()
            distance_goal = math.sqrt( pose['x'] * pose['x'] + pose['y'] * pose['y'])
            self.assertLess(distance_goal, 1.0)
            self.assertGreater(distance_goal, 0.5)

    def test_waypoint_services(self):

        with Morse() as simu:
            # Read the start position, it must be (0.0, 0.0, 0.0)
            pose_stream = simu.robot.pose
            pose = pose_stream.get()

            for key, coord in pose.items():
                if key != 'timestamp':
                    self.assertAlmostEqual(coord, 0.0, delta=0.02)

            logger.info("Moving 2m ahead...")

            simu.robot.motion.goto(2.0, 0.0, 0.0, 0.1, 1.0).result() # wait for completion

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 2.0, delta=0.1)
            self.assertAlmostEqual(pose['y'], 0.0, delta=0.1)
            logger.info("Ok.")

            action = simu.robot.motion.goto(4.0, 0.0, 0.0, 0.1, 1.0) # do not wait for completion
            logger.info("Moving for 1 sec...")
            simu.sleep(1)

            pose = pose_stream.get() #should have done 1m
            self.assertAlmostEqual(pose['x'], 3.0, delta=0.15)
            logger.info("Ok, reached correct position")

            self.assertTrue(action.running())
            self.assertFalse(action.done())

            logger.info("Cancelling motion and waiting for 0.5 sec...")
            action.cancel()
            simu.sleep(0.1)

            self.assertFalse(action.running())
            self.assertTrue(action.done())

            with self.assertRaises(MorseServicePreempted):
                action.result()

            simu.sleep(0.5)
            pose = pose_stream.get() #should not have moved
            self.assertAlmostEqual(pose['x'], 3.0, delta=0.15)
            logger.info("Ok, did not move")

            logger.info("Moving again, waiting for 2 sec, and ensuring the action terminate")
            action = simu.robot.motion.goto(4.0, 0.0, 0.0, 0.1, 1.0) # do not wait for completion
            simu.sleep(2)
            self.assertTrue(action.done())
            self.assertFalse(action.running())

            # Stop will stop the robot, but do not erase current goal
            action = simu.robot.motion.goto(6.0, 0.0, 0.0, 0.1, 1.0) # do not wait for completion
            logger.info("Moving for 1 sec...")
            simu.sleep(1)

            self.assertFalse(action.done())
            self.assertTrue(action.running())

            status = simu.robot.motion.get_status().result()
            self.assertEqual(status, "Transit")

            simu.robot.motion.stop().result()

            # Stop does not change the fact that the goto is pending,
            # but stop the move
            self.assertFalse(action.done())
            self.assertTrue(action.running())

            status = simu.robot.motion.get_status().result()
            self.assertEqual(status, "Stop")

            simu.sleep(0.2)

            pose = pose_stream.get() 
            _x = pose['x']
            self.assertAlmostEqual(pose['x'], 5.2, delta=0.20)

            simu.sleep(0.5)
            pose = pose_stream.get() #should not have moved
            self.assertAlmostEqual(pose['x'], _x, delta=0.03)
            logger.info("Ok, did not move")

            # now resume the move

            simu.robot.motion.resume().result()

            simu.sleep(0.5)

            # must move now
            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 5.7, delta=0.20)
            status = simu.robot.motion.get_status().result()
            self.assertEqual(status, "Transit")

            # wait for the end of the move
            simu.sleep(1.5)
            self.assertTrue(action.done())
            self.assertFalse(action.running())

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 6.0, delta=0.15)
            status = simu.robot.motion.get_status().result()
            self.assertEqual(status, "Arrived")


########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(Waypoints_Test)
