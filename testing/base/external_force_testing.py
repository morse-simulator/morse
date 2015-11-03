#! /usr/bin/env python
"""
This script tests some of the base functionalities of MORSE.
"""

import sys
import math
from morse.testing.testing import MorseTestCase
from pymorse import Morse

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

def send_force_x(actuator, fx):
    actuator.publish({'force': [fx, 0.0, 0.0], 'torque': [0.0, 0.0, 0.0]})

class ExternalForceTest(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        robot_ref = ATRV()

        force = ForceTorque()
        robot_ref.append(force)
        force.add_stream('socket')

        accel = Accelerometer()
        robot_ref.append(accel)
        accel.add_stream('socket')

        robot = ATRV()
        robot.translate(y = 5.0)

        force2 = ForceTorque()
        robot.append(force2)
        force2.name = 'force'
        force2.add_stream('socket')

        accel2 = Accelerometer()
        robot.append(accel2)
        accel2.name = 'accel'
        accel2.add_stream('socket')

        wind = ExternalForce()
        robot.append(wind)
        wind.add_stream('socket')

        env = Environment('empty', fastmode = True)


    def test_external_force(self):
        """ 
        The idea between the test is to verify that a robot against
        the wind is going slower than another one, using the same force.
        The acceleration stay positive in both case.

        We should also verify that the acceleration is different (and
        should be constant), but it seems there is one issue in the
        acceleration computation.
        """
        with Morse() as morse:
            delta = 0.20

            force_ref = morse.robot_ref.force
            accel_ref = morse.robot_ref.accel
            force = morse.robot.force
            accel = morse.robot.accel
            wind = morse.robot.wind

            morse.sleep(0.01)

            acc_ref = accel_ref.get()
            acc = accel.last()

            self.assertAlmostEqual(acc['velocity'][0], 0.0, delta = delta)
            self.assertAlmostEqual(acc['velocity'][1], 0.0, delta = delta)
            self.assertAlmostEqual(acc_ref['velocity'][0], 0.0, delta = delta)
            self.assertAlmostEqual(acc_ref['velocity'][1], 0.0, delta = delta)

            send_force_x(force_ref, 100)
            send_force_x(force, 100)

            morse.sleep(1)
            acc_ref = accel_ref.get()
            acc = accel.last()

            self.assertAlmostEqual(acc['velocity'][0], 2.5, delta = delta)
            self.assertAlmostEqual(acc['velocity'][1], 0.0, delta = delta)
            self.assertAlmostEqual(acc_ref['velocity'][0], 2.5, delta = delta)
            self.assertAlmostEqual(acc_ref['velocity'][1], 0.0, delta = delta)

            send_force_x(wind, -30.0)

            morse.sleep(1)
            acc_ref = accel_ref.get()
            acc = accel.last()

            self.assertAlmostEqual(acc['velocity'][0], 3.25, delta = delta)
            self.assertAlmostEqual(acc['velocity'][1], 0.0, delta = delta)
            self.assertAlmostEqual(acc_ref['velocity'][0], 5.0, delta = delta)

            pass



########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(ExternalForceTest)
