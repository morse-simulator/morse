#! /usr/bin/env python
"""
This script tests some of the base functionalities of MORSE.
"""

import sys
from time import sleep, time
from morse.testing.testing import MorseTestCase
from pymorse import Morse

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

def send_speed(s, v, w):
    s.publish({'v' : v, 'w' : w})

class Velocity_Test(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        robot = ATRV()

        motion = MotionVW()
        motion.properties(ControlType = 'Velocity')
        robot.append(motion)
        motion.add_stream('socket')

        vel = Velocity()
        robot.append(vel)
        vel.add_stream('socket')

        teleport = Teleport()
        robot.append(teleport)
        teleport.add_stream('socket')
        
        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def expect_value(self, linear_velocity,
                           angular_velocity,
                           world_linear_velocity):

        precision=0.1

        means = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # compute the mean on number_iteration (here 60, so normally
        # something like 1 sec). As it is a physical controller, there
        # is no guarantee that at each instant, we have exactly the
        # expected speed, but, in mean, we must have the requested
        # value.
        number_iteration = 60
        for i in range(0, number_iteration):
            vel = self.vel_stream.get()
            for j in range(3):
                means[j] += vel['linear_velocity'][j]
            for j in range(3):
                means[j+3] += vel['angular_velocity'][j]
            for j in range(3):
                means[j+6] += vel['world_linear_velocity'][j]

        for i in range(9):
            means[i] = means[i] / number_iteration

        expected = linear_velocity + angular_velocity + world_linear_velocity

        for i in range(9):
            self.assertAlmostEqual(means[i], expected[i], delta=precision)

    def test_vw_controller(self):
        with Morse() as simu:

            self.vel_stream = simu.robot.vel
            v_w = simu.robot.motion

            simu.deactivate('robot.teleport')

            # wait a few sec that physics stop its fun
            sleep(0.5)
            self.expect_value([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

            send_speed(v_w, 1.0, 0.0)
            sleep(0.5)
            self.expect_value([1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [1.0, 0.0, 0.0])

            send_speed(v_w, 0.0, 0.0)
            sleep(0.5)
            self.expect_value([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

            send_speed(v_w, 0.0, math.pi / 4.0)
            sleep(0.5)
            self.expect_value([0.0, 0.0, 0.0], [0.0, 0.0, math.pi / 4.0], [0.0, 0.0, 0.0])

            send_speed(v_w, 0.0, 0.0)
            sleep(0.1)

            simu.deactivate('robot.motion')
            simu.activate('robot.teleport')
            simu.robot.teleport.publish({'x' : 1.0, 'y' : 0.0, 'z': 0.0,
                                         'yaw': math.pi/2, 'pitch': 0.0,
                                         'roll': 0.0})
            sleep(0.1)
            simu.deactivate('robot.teleport')
            simu.activate('robot.motion')
            sleep(0.1)

            send_speed(v_w, 1.0, 0.0)
            sleep(0.5)
            self.expect_value([1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 1.0, 0.0])





########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(Velocity_Test)

