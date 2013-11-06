#! /usr/bin/env python
"""
This script tests the SICK laser range sensor in MORSE.
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

class Sick_Test(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        robot = ATRV()
        robot.rotate(z = math.pi)
        robot.translate(x = -4.5)

        sick = Sick('Sick')
        sick.translate(z=0.9)
        sick.properties(laser_range = 10.0, Visible_arc = False)
        sick.create_laser_arc()
        robot.append(sick)
        sick.add_stream('socket')

        env = Environment('indoors-1/boxes', fastmode = True)
        env.add_service('socket')


    def test_sick(self):
        """ This test is guaranteed to be started only when the simulator
        is ready.
        """
        with Morse() as morse:
        
            # Read the data from the sick sensor
            self.sick_stream = morse.robot.Sick

            sick = self.sick_stream.get()

            # On the right of the sensor, nothing to hit. So position is
            # (0.0, 0.0, 0.0) and distance == laser_range
            for index in range(105, 180):
                ray = sick['point_list'][index]
                self.assertAlmostEqual(ray[0], 0.0)
                self.assertAlmostEqual(ray[1], 0.0)
                self.assertAlmostEqual(ray[2], 0.0)
                length = sick['range_list'][index]
                self.assertAlmostEqual(length, 10.0)

            # In the center of the sensor, we hit the red block (situed
            # near (-7, 0)).
            # Distance to hit is near 2.5
            for index in range(80, 100):
                length = sick['range_list'][index]
                self.assertAlmostEqual(length, 2.5, delta=0.05)

            # Check some specific point. Hit 90 is the ray corresponding
            # to angle math.pi/2 (in front of the robot). y is computed
            # by trigonometry.
            ray = sick['point_list'][90]
            self.assertAlmostEqual(ray[0], 2.5, delta=0.05)
            self.assertAlmostEqual(ray[1], 0.0, delta=0.05)
            self.assertAlmostEqual(ray[2], 0.0, delta=0.05)

            ray = sick['point_list'][85]
            self.assertAlmostEqual(ray[0], 2.5, delta=0.05)
            self.assertAlmostEqual(ray[1], -2.5 * math.tan(math.radians(5)),
                                   delta=0.05)
            self.assertAlmostEqual(ray[2], 0.0, delta=0.05)

            ray = sick['point_list'][95]
            self.assertAlmostEqual(ray[0], 2.5, delta=0.05)
            self.assertAlmostEqual(ray[1], -2.5 * math.tan(math.radians(-5)),
                                   delta=0.05)
            self.assertAlmostEqual(ray[2], 0.0, delta=0.05)


            # Then, there is a full empty sector
            for index in range(105, 150):
                ray = sick['point_list'][index]
                self.assertAlmostEqual(ray[0], 0.0)
                self.assertAlmostEqual(ray[1], 0.0)
                self.assertAlmostEqual(ray[2], 0.0)
                length = sick['range_list'][index]
                self.assertAlmostEqual(length, 10.0)

            # The last ray hit the green block
            # Distance and real position are a bit complicated to
            # compute manually, so don't check real precision. Just
            # verify that the distance is near 5.8
            #print([i for i,r in enumerate(sick['range_list']) if 5<r<6])
            for index in range(16, 22):
                length = sick['range_list'][index]
                self.assertAlmostEqual(length, 5.8, delta=0.15)



########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(Sick_Test)
