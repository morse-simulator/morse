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
        
        motion = MotionVW()
        robot.append(motion)
        
        pose = Pose()
        robot.append(pose)
        pose_noised = Pose()
        pose_noised.alter('Noise', pos_std=1, rot_std=1)
        robot.append(pose_noised)
        
        gps = GPS()
        robot.append(gps)
        gps_noised = GPS()
        gps_noised.alter('Noise', pos_std=1)
        robot.append(gps_noised)
        
        gyroscope = Gyroscope()
        robot.append(gyroscope)
        gyroscope_noised = Gyroscope()
        gyroscope_noised.alter('Noise', rot_std=1)
        robot.append(gyroscope_noised)
        
        odometry = Odometry()
        robot.append(odometry)
        odometry.level('integrated')
        odometry_noised = Odometry()
        odometry_noised.alter('Noise', pos_std=1, rot_std=1)
        robot.append(odometry_noised)
        odometry_noised.level('integrated')
        
        robot.add_default_interface('socket')
        env = Environment('empty', fastmode = True)
        env.add_interface('socket')
        
    def test_noised_pose(self):
        """ Test if the Pose data is noised
        """
        with Morse() as morse:
            d = morse.robot.pose.get()
            dn = morse.robot.pose_noised.get()
            for i in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
                self.assertNotAlmostEqual(d[i], dn[i], delta=.001)

    def test_noised_gyro(self):
        """ Test if the Gyroscope data is noised
        """
        with Morse() as morse:
            d = morse.robot.gyroscope.get()
            dn = morse.robot.gyroscope_noised.get()
            for i in ['roll', 'pitch', 'yaw']:
                self.assertNotAlmostEqual(d[i], dn[i], delta=.001)

    def test_noised_gps(self):
        """ Test if the GPS data is noised
        """
        with Morse() as morse:
            d = morse.robot.gps.get()
            dn = morse.robot.gps_noised.get()
            for i in ['x', 'y', 'z']:
                self.assertNotAlmostEqual(d[i], dn[i], delta=.001)
                          
    def test_noised_odometry(self):
        """ Test if the odometry data is noised
        """
        with Morse() as morse:
            morse.robot.motion.publish({'v': 1, 'w':.5})
            morse.sleep(1)
            d = morse.robot.odometry.get()
            dn = morse.robot.odometry_noised.get()
            for i in ['x', 'y', 'z', 'yaw', 'pitch', 'roll']:
                #, 'vx', 'vy', 'vz', 'wz', 'wy', 'wz']:
                self.assertNotAlmostEqual(d[i], dn[i], delta=.001)

    
########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(NoiseTest)
