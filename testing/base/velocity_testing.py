#! /usr/bin/env python
"""
This script tests some of the base functionalities of MORSE.
"""

import sys
import math
from morse.testing.testing import MorseMoveTestCase
from pymorse import Morse

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

class Velocity_Test(MorseMoveTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        robot = ATRV()

        motion = MotionXYW()
        robot.append(motion)
        motion.add_stream('socket')
        motion.add_service('socket')

        vel = Velocity()
        robot.append(vel)
        vel.add_stream('socket')

        vel_pos = Velocity()
        robot.append(vel_pos)
        vel_pos.add_stream('socket')
        vel_pos.properties(ComputationMode = 'Position')

        vel_pi = Velocity()
        robot.append(vel_pi)
        vel_pi.rotate(z = math.pi / 2, y = math.pi / 2)
        vel_pi.add_stream('socket')

        vel_pos_pi = Velocity()
        robot.append(vel_pos_pi)
        vel_pos_pi.rotate(z = math.pi / 2, y = math.pi / 2)
        vel_pos_pi.add_stream('socket')
        vel_pos_pi.properties(ComputationMode = 'Position')
        
        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def assert_velocity(self, morse, command, expected):
        delta = 0.1

        self.xyw.publish(command)
        morse.sleep(0.1)

        vel = self.vel_stream.last()
        vel_pos = self.vel_pos_stream.last()
        vel_pi = self.vel_pi_stream.last()
        vel_pos_pi = self.vel_pos_pi_stream.last()

        self.assertAlmostEqual(vel['linear_velocity'][0], expected[0], delta = delta)
        self.assertAlmostEqual(vel['linear_velocity'][1], expected[1], delta = delta)
        self.assertAlmostEqual(vel['linear_velocity'][2], expected[2], delta = delta)

        self.assertAlmostEqual(vel['angular_velocity'][0], expected[3], delta = delta)
        self.assertAlmostEqual(vel['angular_velocity'][1], expected[4], delta = delta)
        self.assertAlmostEqual(vel['angular_velocity'][2], expected[5], delta = delta)

        self.assertAlmostEqual(vel['world_linear_velocity'][0], expected[0], delta = delta)
        self.assertAlmostEqual(vel['world_linear_velocity'][1], expected[1], delta = delta)
        self.assertAlmostEqual(vel['world_linear_velocity'][2], expected[2], delta = delta)

        self.assertAlmostEqual(vel_pos['linear_velocity'][0], expected[0], delta = delta)
        self.assertAlmostEqual(vel_pos['linear_velocity'][1], expected[1], delta = delta)
        self.assertAlmostEqual(vel_pos['linear_velocity'][2], expected[2], delta = delta)

        self.assertAlmostEqual(vel_pos['angular_velocity'][0], expected[3], delta = delta)
        self.assertAlmostEqual(vel_pos['angular_velocity'][1], expected[4], delta = delta)
        self.assertAlmostEqual(vel_pos['angular_velocity'][2], expected[5], delta = delta)

        self.assertAlmostEqual(vel_pos['world_linear_velocity'][0], expected[0], delta = delta)
        self.assertAlmostEqual(vel_pos['world_linear_velocity'][1], expected[1], delta = delta)
        self.assertAlmostEqual(vel_pos['world_linear_velocity'][2], expected[2], delta = delta)

        self.assertAlmostEqual(vel_pi['linear_velocity'][0], expected[2], delta = delta)
        self.assertAlmostEqual(vel_pi['linear_velocity'][1], - expected[0], delta = delta)
        self.assertAlmostEqual(vel_pi['linear_velocity'][2], expected[1], delta = delta)

        self.assertAlmostEqual(vel_pi['angular_velocity'][0], - expected[5], delta = delta)
        self.assertAlmostEqual(vel_pi['angular_velocity'][1], expected[3], delta = delta)
        self.assertAlmostEqual(vel_pi['angular_velocity'][2], expected[2], delta = delta)

        self.assertAlmostEqual(vel_pi['world_linear_velocity'][0], expected[0], delta = delta)
        self.assertAlmostEqual(vel_pi['world_linear_velocity'][1], expected[1], delta = delta)
        self.assertAlmostEqual(vel_pi['world_linear_velocity'][2], expected[2], delta = delta)

        self.assertAlmostEqual(vel_pos_pi['linear_velocity'][0], expected[2], delta = delta)
        self.assertAlmostEqual(vel_pos_pi['linear_velocity'][1], - expected[0], delta = delta)
        self.assertAlmostEqual(vel_pos_pi['linear_velocity'][2], expected[1], delta = delta)

        self.assertAlmostEqual(vel_pos_pi['angular_velocity'][0], - expected[5], delta = delta)
        self.assertAlmostEqual(vel_pos_pi['angular_velocity'][1], expected[3], delta = delta)
        self.assertAlmostEqual(vel_pos_pi['angular_velocity'][2], expected[2], delta = delta)

        self.assertAlmostEqual(vel_pos_pi['world_linear_velocity'][0], expected[0], delta = delta)
        self.assertAlmostEqual(vel_pos_pi['world_linear_velocity'][1], expected[1], delta = delta)
        self.assertAlmostEqual(vel_pos_pi['world_linear_velocity'][2], expected[2], delta = delta)

    def test_velocity_sensor(self):
        with Morse() as morse:

            self.xyw = morse.robot.motion
            self.vel_stream = morse.robot.vel
            self.vel_pos_stream = morse.robot.vel_pos
            self.vel_pi_stream = morse.robot.vel_pi
            self.vel_pos_pi_stream = morse.robot.vel_pos_pi

            self.assert_velocity(morse, {"x": 1.0, "y": 0.0, "w": 0.0},
                                 [1.0, 0.0, 0.0, 0.0, 0.0, 0.0])

            self.assert_velocity(morse, {"x": 0.0, "y": 0.0, "w": 0.0},
                                 [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

            self.assert_velocity(morse, {"x": 0.0, "y": -1.0, "w": 0.0},
                                 [0.0, -1.0, 0.0, 0.0, 0.0, 0.0])

            self.assert_velocity(morse, {"x": 0.0, "y": 0.0, "w": 0.0},
                                 [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

            self.assert_velocity(morse, {"x": 1.0, "y": 0.0, "w": 1.0},
                                 [1.0, 0.0, 0.0, 0.0, 0.0, 1.0])

            self.assert_velocity(morse, {"x": 0.0, "y": 0.0, "w": 0.0},
                                 [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(Velocity_Test)
