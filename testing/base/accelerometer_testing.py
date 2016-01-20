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

class Accelero_Test(MorseMoveTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        robot = ATRV()

        motion = MotionXYW()
        robot.append(motion)
        motion.add_stream('socket')
        motion.add_service('socket')

        accel = Accelerometer()
        robot.append(accel)
        accel.add_stream('socket')

        accel_pi = Accelerometer()
        robot.append(accel_pi)
        accel_pi.rotate(z = math.pi / 2)
        accel_pi.add_stream('socket')
        
        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    # The Blender control is not 'perfect' and the speed is so not
    # perfectly constant, leading to 'small' acceleration.
    def assert_accel_almost_null(self, acc, idx):
        self.assertGreater(acc['acceleration'][idx], -1.2)
        self.assertLess(acc['acceleration'][idx], 1.2)

    def test_accel_sensor(self):
        with Morse() as morse:

            delta = 0.06

            xyw = morse.robot.motion
            accel_stream = morse.robot.accel
            accel_pi_stream = morse.robot.accel_pi

            xyw.publish({'x' : 1.0, 'y': 0.0, 'w' : 0.0})
            morse.sleep(0.01)

            acc = accel_stream.get() 
            acc_pi = accel_pi_stream.last() 

            self.assertGreater(acc['acceleration'][0], 50)
            self.assert_accel_almost_null(acc, 1)
            self.assertAlmostEqual(acc['velocity'][0], 1.0, delta = delta)
            self.assertAlmostEqual(acc['velocity'][1], 0.0, delta = delta)

            # acceleration phase
            self.assertLess(acc_pi['acceleration'][1], -50)
            self.assert_accel_almost_null(acc_pi, 0)
            self.assertAlmostEqual(acc_pi['velocity'][0], 0.0, delta = delta)
            self.assertAlmostEqual(acc_pi['velocity'][1], -1.0, delta = delta)

            morse.sleep(0.1)

            acc = accel_stream.get() 
            acc_pi = accel_pi_stream.get() 
            self.assert_accel_almost_null(acc, 0)
            self.assert_accel_almost_null(acc, 1)
            self.assertAlmostEqual(acc['velocity'][0], 1.0, delta = delta)
            self.assertAlmostEqual(acc['velocity'][1], 0.0, delta = delta)
            self.assertAlmostEquals(acc['distance'], 1/60, delta = delta)

            self.assert_accel_almost_null(acc_pi, 0)
            self.assert_accel_almost_null(acc_pi, 1)
            self.assertAlmostEqual(acc_pi['velocity'][1], -1.0, delta = delta)
            self.assertAlmostEqual(acc_pi['velocity'][0], 0.0, delta = delta)
            self.assertAlmostEquals(acc_pi['distance'], 1/60, delta = delta)

            xyw.publish({'x' : 0.0, 'y': 0.0, 'w' : 0.0})
            morse.sleep(0.01)

            # decacceleration phase
            acc = accel_stream.last() 
            acc_pi = accel_pi_stream.last() 

            self.assertLess(acc['acceleration'][0], -50)
            self.assert_accel_almost_null(acc, 1)
            self.assertAlmostEqual(acc['velocity'][0], 0.0, delta = delta)
            self.assertAlmostEqual(acc['velocity'][1], 0.0, delta = delta)
            self.assertAlmostEquals(acc['distance'], 0.0, delta = delta)

            self.assert_accel_almost_null(acc_pi, 0)
            self.assertGreater(acc_pi['acceleration'][1], 50)
            self.assertAlmostEqual(acc_pi['velocity'][0], 0.0, delta = delta)
            self.assertAlmostEqual(acc_pi['velocity'][1], 0.0, delta = delta)
            self.assertAlmostEquals(acc_pi['distance'], 0.0, delta = delta)


            morse.sleep(0.1)
            acc = accel_stream.get() 
            acc_pi = accel_pi_stream.get() 
            self.assert_accel_almost_null(acc, 0)
            self.assert_accel_almost_null(acc, 1)
            self.assertAlmostEqual(acc['velocity'][0], 0.0, delta = delta)
            self.assertAlmostEqual(acc['velocity'][1], 0.0, delta = delta)
            self.assertAlmostEquals(acc['distance'], 0.0, delta = delta)

            self.assert_accel_almost_null(acc_pi, 0)
            self.assert_accel_almost_null(acc_pi, 1)
            self.assertAlmostEqual(acc_pi['velocity'][0], 0.0, delta = delta)
            self.assertAlmostEqual(acc_pi['velocity'][1], 0.0, delta = delta)
            self.assertAlmostEquals(acc_pi['distance'], 0.0, delta = delta)


            xyw.publish({'x' : 0.0, 'y': 1.0, 'w' : 0.0})

            morse.sleep(0.1)
            acc = accel_stream.get() 
            acc_pi = accel_pi_stream.get() 
            self.assert_accel_almost_null(acc, 0)
            self.assert_accel_almost_null(acc, 1)
            self.assertAlmostEqual(acc['velocity'][0], 0.0, delta = delta)
            self.assertAlmostEqual(acc['velocity'][1], 1.0, delta = delta)
            self.assertAlmostEquals(acc['distance'], 1/60, delta = delta)

            self.assert_accel_almost_null(acc_pi, 0)
            self.assert_accel_almost_null(acc_pi, 1)
            self.assertAlmostEqual(acc_pi['velocity'][0], 1.0, delta = delta)
            self.assertAlmostEqual(acc_pi['velocity'][1], 0.0, delta = delta)
            self.assertAlmostEquals(acc_pi['distance'], 1/60, delta = delta)


########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(Accelero_Test)
