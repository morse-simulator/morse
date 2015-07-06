#! /usr/bin/env python
"""
This script tests the several Noise modifiers
"""

from morse.testing.testing import MorseTestCase

try:
    # Include this import to be able to use your test file as a regular 
    # builder script, ie, usable with: 'morse [run|exec] <your test>.py
    from morse.builder import *
except ImportError:
    pass

import os
import sys
from pymorse import Morse


class NoiseTest(MorseTestCase):

    def setUpEnv(self):
        
        robot = ATRV()
                
        imu = IMU()
        robot.append(imu)
        imu_noised = IMU()
        imu_noised.alter('Noise', gyro_std=1, accel_std=1)
        robot.append(imu_noised)
        
        robot.add_default_interface('socket')
        env = Environment('empty', fastmode = True)
        env.add_interface('socket')
        env.properties(longitude = 43.6, latitude = 1.4333, altitude = 135.0)
        
    def test_noised_imu(self):
        """ Test if the IMU data is noised
        """
        with Morse() as morse:
            d = morse.robot.imu.get()
            dn = morse.robot.imu_noised.get()
            for i in ['angular_velocity', 'linear_acceleration']:
                for j in range(0,3):
                    self.assertNotAlmostEqual(d[i][j], dn[i][j], delta=.001)
              
########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(NoiseTest)
