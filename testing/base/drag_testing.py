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

class DragTest(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        robot_ref = ATRV()
        # Make sure there is no friction with ground and it won't fall
        robot_ref.translate(z = 1.0)
        robot_ref._bpy_object.game.lock_location_z = True

        force = ForceTorque()
        robot_ref.append(force)
        force.add_stream('socket')

        accel = Accelerometer()
        robot_ref.append(accel)
        accel.add_stream('socket')

        robot = ATRV()
        robot.translate(y = 5.0)
        # Make sure there is no friction with ground and it won't fall
        robot.translate(z = 1.0)
        robot._bpy_object.game.lock_location_z = True

        force2 = ForceTorque()
        robot.append(force2)
        force2.name = 'force'
        force2.add_stream('socket')

        accel2 = Accelerometer()
        robot.append(accel2)
        accel2.name = 'accel'
        accel2.add_stream('socket')

        drag = Drag()
        # Looks like a bad brick but want to clearly demonstrate drag
        # effect
        drag.properties(DragCoeffX = 1.0)
        robot.append(drag)

        env = Environment('empty', fastmode = True)


    def test_drag(self):
        """ 
        The idea between the test is to verify that a robot against
        the wind is going slower than another one, using the same force.
        The acceleration stay positive in both case.
        """
        with Morse() as morse:
            delta = 0.20

            force_ref = morse.robot_ref.force
            accel_ref = morse.robot_ref.accel
            force = morse.robot.force
            accel = morse.robot.accel

            morse.sleep(0.01)

            acc_ref = accel_ref.get()
            acc = accel.last()

            self.assertAlmostEqual(acc['acceleration'][0], 0.0, delta = delta)
            self.assertAlmostEqual(acc['acceleration'][1], 0.0, delta = delta)
            self.assertAlmostEqual(acc_ref['acceleration'][0], 0.0, delta = delta)
            self.assertAlmostEqual(acc_ref['acceleration'][1], 0.0, delta = delta)
            self.assertAlmostEqual(acc['velocity'][0], 0.0, delta = delta)
            self.assertAlmostEqual(acc['velocity'][1], 0.0, delta = delta)
            self.assertAlmostEqual(acc_ref['velocity'][0], 0.0, delta = delta)
            self.assertAlmostEqual(acc_ref['velocity'][1], 0.0, delta = delta)

            send_force_x(force_ref, 100)
            send_force_x(force, 100)

            morse.sleep(1)
            acc_ref = accel_ref.get()
            acc = accel.last()

            # After 1 sec, the speed of robot_ref should be less than
            # the speed of robot due to the drag effect 

            self.assertAlmostEqual(acc['acceleration'][0], 3.80, delta = delta)
            self.assertAlmostEqual(acc['acceleration'][1], 0.0, delta = delta)
            self.assertAlmostEqual(acc_ref['acceleration'][0], 4.80, delta = delta)
            self.assertAlmostEqual(acc_ref['acceleration'][1], 0.0, delta = delta)
            self.assertAlmostEqual(acc['velocity'][0], 4.6, delta = delta)
            self.assertAlmostEqual(acc['velocity'][1], 0.0, delta = delta)
            self.assertAlmostEqual(acc_ref['velocity'][0], 5.0, delta = delta)
            self.assertAlmostEqual(acc_ref['velocity'][1], 0.0, delta = delta)

            morse.sleep(1)
            acc_ref = accel_ref.get()
            acc = accel.last()

            # After one more second, the differnece between both speed
            # increases. acceleration of robot_ref is constant, while
            # acceleration of robot decrease (as drag effect increases
            # with speed)

            self.assertAlmostEqual(acc['acceleration'][0], 2.0, delta = delta)
            self.assertAlmostEqual(acc_ref['acceleration'][0], 4.80, delta = delta)
            self.assertAlmostEqual(acc['velocity'][0], 7.5, delta = delta)
            self.assertAlmostEqual(acc_ref['velocity'][0], 10.0, delta = delta)

            # When no force are applied anymore, the robot_ref will
            # continue at constant speed (well, it is is supposed too,
            # but there seems to have some friction somewhere :D)
            # while the robot speed will slowly decreases until stop
            send_force_x(force_ref, 0)
            send_force_x(force, 0)

            morse.sleep(1)
            acc_ref = accel_ref.get()
            acc = accel.last()

            self.assertAlmostEqual(acc['acceleration'][0], -1.6, delta = delta)
            self.assertAlmostEqual(acc_ref['acceleration'][0], -0.2, delta = delta)
            self.assertAlmostEqual(acc['velocity'][0], 5.30, delta = delta)
            self.assertAlmostEqual(acc_ref['velocity'][0], 9.80, delta = delta)

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(DragTest)
