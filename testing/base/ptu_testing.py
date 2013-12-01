#! /usr/bin/env python
"""
This script tests the 'data stream' oriented feature of the socket interface.
"""

from morse.testing.testing import MorseTestCase

try:
    # Include this import to be able to use your test file as a regular 
    # builder script, ie, usable with: 'morse [run|exec] <your test>.py
    from morse.builder import *
except ImportError:
    pass

import sys
import math
from pymorse import Morse

def send_angles(s, pan, tilt):
    s.publish({'pan' : pan, 'tilt' : tilt})

class PTUTest(MorseTestCase):

    def setUpEnv(self):

        ptu_x = 0.2020
        ptu_z = 1.4400

        robot = ATRV()

        ptu = PTU()
        ptu.add_stream('socket')
        ptu.translate(x=ptu_x, z=ptu_z)
        ptu.add_service('socket')
        ptu.properties(Speed = 0.5)
        robot.append(ptu)

        posture = PTUPosture()
        posture.add_stream('socket')
        ptu.append(posture)

        gyro = Gyroscope()
        gyro.add_stream('socket')
        ptu.append(gyro)
        
        chair = PassiveObject(prefix='RollingChair')
        chair.translate(x=ptu_x, y=3, z=0.01)

        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def test_datastream(self):
        """ Test if we can connect to the pose data stream, and read from it.
        """

        with Morse() as morse:
            gyro_stream = morse.robot.ptu.gyro
            posture_stream = morse.robot.ptu.posture
            ptu_stream = morse.robot.ptu

            angles = gyro_stream.get()
            posture = posture_stream.get()

            precision = 0.02
            moving_precision = 0.1

            self.assertAlmostEqual(posture['pan'], 0.0, delta=precision)
            self.assertAlmostEqual(posture['tilt'], 0.0, delta=precision)
            self.assertAlmostEqual(angles['yaw'], 0.0, delta=precision)
            self.assertAlmostEqual(angles['pitch'], 0.0, delta=precision)

            send_angles(ptu_stream, 1.0, 0.0)
            morse.sleep(1.0)

            # here at speed of 0.5 rad / sec, we must be at the middle
            # of the trip, check it :)
            angles = gyro_stream.get()
            posture = posture_stream.get()
            self.assertAlmostEqual(posture['pan'], 0.5, delta=moving_precision)
            self.assertAlmostEqual(posture['tilt'], 0.0, delta=moving_precision)
            self.assertAlmostEqual(angles['yaw'], 0.5, delta=moving_precision)
            self.assertAlmostEqual(angles['pitch'], 0.0, delta=moving_precision)

            morse.sleep(1.0)
            # now we must have achieve ptu rotation
            angles = gyro_stream.get()
            posture = posture_stream.get()

            self.assertAlmostEqual(posture['pan'], 1.0, delta=precision)
            self.assertAlmostEqual(posture['tilt'], 0.0, delta=precision)
            self.assertAlmostEqual(angles['yaw'], 1.0, delta=precision)
            self.assertAlmostEqual(angles['pitch'], 0.0, delta=precision)

            send_angles(ptu_stream, 1.0, -1.0)
            morse.sleep(2.0)
            angles = gyro_stream.get()
            posture = posture_stream.get()

            self.assertAlmostEqual(posture['pan'], 1.0, delta=precision)
            self.assertAlmostEqual(posture['tilt'], -1.0, delta=precision)
            self.assertAlmostEqual(angles['yaw'], 1.0, delta=precision)
            self.assertAlmostEqual(angles['pitch'], -1.0, delta=precision)

            send_angles(ptu_stream, 0.0, 0.0)
            morse.sleep(2.0)
            angles = gyro_stream.get()
            posture = posture_stream.get()

            self.assertAlmostEqual(posture['pan'], 0.0, delta=precision)
            self.assertAlmostEqual(posture['tilt'], 0.0, delta=precision)
            self.assertAlmostEqual(angles['yaw'], 0.0, delta=precision)
            self.assertAlmostEqual(angles['pitch'], 0.0, delta=precision)

    def test_set_service(self):
        """ Test if we can connect to the pose data stream, and read from it.
        """

        with Morse() as morse:
            gyro_stream = morse.robot.ptu.gyro
            posture_stream = morse.robot.ptu.posture
            ptu_stream = morse.robot.ptu

            angles = gyro_stream.get()
            posture = posture_stream.get()

            precision = 0.02
            moving_precision = 0.1

            self.assertAlmostEqual(posture['pan'], 0.0, delta=precision)
            self.assertAlmostEqual(posture['tilt'], 0.0, delta=precision)
            self.assertAlmostEqual(angles['yaw'], 0.0, delta=precision)
            self.assertAlmostEqual(angles['pitch'], 0.0, delta=precision)

            res = morse.rpc('robot.ptu', 'get_pan_tilt')
            self.assertAlmostEqual(res[0], 0.0, delta=precision)
            self.assertAlmostEqual(res[1], 0.0, delta=precision)

            morse.rpc('robot.ptu', 'set_pan_tilt', 1.0, 0.0)

            angles = gyro_stream.get()
            posture = posture_stream.get()
            self.assertAlmostEqual(posture['pan'], 1.0, delta=moving_precision)
            self.assertAlmostEqual(posture['tilt'], 0.0, delta=moving_precision)
            self.assertAlmostEqual(angles['yaw'], 1.0, delta=moving_precision)
            self.assertAlmostEqual(angles['pitch'], 0.0, delta=moving_precision)

            res = morse.rpc('robot.ptu', 'get_pan_tilt')
            self.assertAlmostEqual(res[0], 1.0, delta=precision)
            self.assertAlmostEqual(res[1], 0.0, delta=precision)

    def test_lookat(self):
        """ Test if the PTU can successfully orient itself towards an
        absolute x,y,z position and towards a given object.
        """

        with Morse() as morse:

            #TODO: Stupid duplication of SetUpEnv values. Could not find a way
            #to share the value. Class variables does not seem to work here.
            ptu_x = 0.2020
            ptu_z = 1.4400 + .1 # 0.1 -> height of ATRV center

            precision = 0.02

            res = morse.rpc('robot.ptu', 'look_at_point', 1 ,0 ,ptu_z)
            res = morse.rpc('robot.ptu', 'get_pan_tilt')
            self.assertAlmostEqual(res[0], 0.0, delta=precision)
            self.assertAlmostEqual(res[1], 0.0, delta=precision)



            res = morse.rpc('robot.ptu', 'look_at_point', -1 ,0 ,ptu_z)
            res = morse.rpc('robot.ptu', 'get_pan_tilt')
#            self.assertAlmostEqual(res[0], math.radians(180.0), delta=precision)
            self.assertAlmostEqual(res[1], 0.0, delta=precision)



            res = morse.rpc('robot.ptu', 'look_at_point', ptu_x,1,ptu_z)
            res = morse.rpc('robot.ptu', 'get_pan_tilt')
            self.assertAlmostEqual(res[0], math.radians(90), delta=precision)
            self.assertAlmostEqual(res[1], 0.0, delta=precision)



            res = morse.rpc('robot.ptu', 'look_at_point', ptu_x, -1, ptu_z)
            res = morse.rpc('robot.ptu', 'get_pan_tilt')
            self.assertAlmostEqual(res[0], math.radians(-90), delta=precision)
            self.assertAlmostEqual(res[1], 0.0, delta=precision)

            
            
            res = morse.rpc('robot.ptu', 'look_at_point', ptu_x,0,10)
            res = morse.rpc('robot.ptu', 'get_pan_tilt')
            self.assertAlmostEqual(res[1], math.radians(-90), delta=precision)
            # Reset position
            morse.rpc('robot.ptu', 'set_pan_tilt', 0.0, 0.0)

            res = morse.rpc('robot.ptu', 'look_at_object', 'chair')
            res = morse.rpc('robot.ptu', 'get_pan_tilt')
            self.assertAlmostEqual(res[0], math.radians(90), delta=precision)
            self.assertAlmostEqual(res[1], 0.466, delta=precision)



########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(PTUTest)
