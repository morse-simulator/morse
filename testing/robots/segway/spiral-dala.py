#! /usr/bin/env python
"""
This script tests the Dala robot with a v, omega actuator doing a spiral motion
"""

import sys
import socket
import json
import math
from time import sleep
from morse.testing.testing import MorseTestCase
from pymorse import Morse

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

def send_speed(s, v, w, t):
    s.send(json.dumps({'v' : v, 'w' : w}).encode())
    sleep(t)
    s.send(json.dumps({'v' : 0.0, 'w' : 0.0}).encode())
    sleep(1)

def send_service_speed(s, v, w, t):
    s.call_server('Motion_Controller', 'set_speed', v, w)
    sleep(t)
    s.call_server('Motion_Controller', 'stop')
    sleep(1)

class Spiral_Test(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        
        robot = Robot('atrv')
        robot.translate(z=0.1)

        pose = Sensor('pose')
        robot.append(pose)
        pose.configure_mw('socket')
        pose.configure_mw('text')

        motion = Actuator('v_omega')
        robot.append(motion)
        motion.configure_mw('socket')
        motion.configure_service('socket')
        
        env = Environment('indoors-1/indoor-1')
        env.configure_service('socket')

    def test_vw_controller(self):
        with Morse() as morse:
        
            # Read the start position, it must be (0.0, 0.0, 0.0)
            pose_stream = morse.stream('Pose')
            pose = pose_stream.get()
            for key,coord in pose.items():
                if key == 'z':
                    self.assertAlmostEqual(coord, 0.0, delta=0.03)
                else:
                    self.assertAlmostEqual(coord, 0.0, delta=0.03)

            # v_w socket
            port = morse.get_stream_port('Motion_Controller')
            v_w_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            v_w_client.connect(('localhost', port))

            send_speed(v_w_client, 0.0, -math.pi/4.0, 2.0)

            pose = pose_stream.get()
            # for non-null w, we have r = v /  w
            self.assertAlmostEqual(pose['x'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['y'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['z'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['yaw'], -math.pi/2.0, delta=0.15)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=0.15)

            send_speed(v_w_client, 0.0, math.pi/4.0, 2.0)

            pose = pose_stream.get()
            for key,coord in pose.items():
                if key == 'z':
                    self.assertAlmostEqual(coord, 0.0, delta=0.15)
                else:
                    self.assertAlmostEqual(coord, 0.0, delta=0.15)

            # PART ONE

            send_speed(v_w_client, 0.5, -math.pi/8.0, 4.0)

            pose = pose_stream.get()
            # for non-null w, we have r = v /  w
            self.assertAlmostEqual(pose['x'], 4.0/ math.pi , delta=0.20)
            self.assertAlmostEqual(pose['y'], -4.0/ math.pi , delta=0.20)
            self.assertAlmostEqual(pose['z'], 0.0, delta=0.20)
            self.assertAlmostEqual(pose['yaw'], -math.pi/2.0, delta=0.20)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=0.20)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=0.20)

            # PART TWO
           
            send_speed(v_w_client, 0.5, -math.pi/16.0, 8.0)

            """
            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 0.0/ math.pi , delta=0.20)
            self.assertAlmostEqual(pose['y'], -8.0/ math.pi , delta=0.20)
            self.assertAlmostEqual(pose['z'], 0.0, delta=0.20)
            self.assertAlmostEqual(pose['yaw'], -math.pi, delta=0.20)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=0.20)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=0.20)
            """

            send_speed(v_w_client, 0.5, -math.pi/32.0, 16.0)

            """
            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], -4.0/ math.pi , delta=0.20)
            self.assertAlmostEqual(pose['y'], -4.0/ math.pi , delta=0.20)
            self.assertAlmostEqual(pose['z'], 0.0, delta=0.20)
            self.assertAlmostEqual(pose['yaw'], math.pi/2.0, delta=0.20)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=0.20)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=0.20)
            """

    def X_test_vw_service_controller(self):
    #def test_vw_service_controller(self):
        with Morse() as morse:
        
            # Read the start position, it must be (0.0, 0.0, 0.0)
            pose_stream = morse.stream('Pose')
            pose = pose_stream.get()
            for key,coord in pose.items():
                if key == 'z':
                    self.assertAlmostEqual(coord, 0.10, delta=0.02)
                else:
                    self.assertAlmostEqual(coord, 0.0, delta=0.02)

            send_service_speed(morse, 1.0, 0.0, 2.0)

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 2.0, delta=0.15)
            self.assertAlmostEqual(pose['y'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['z'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['yaw'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=0.15)

            send_service_speed(morse, -1.0, 0.0, 2.0)

            pose = pose_stream.get()
            for key,coord in pose.items():
                if key == 'z':
                    self.assertAlmostEqual(coord, 0.10, delta=0.15)
                else:
                    self.assertAlmostEqual(coord, 0.0, delta=0.15)

            send_service_speed(morse, 1.0, -math.pi/4.0, 2.0)

            pose = pose_stream.get()
            print ("POSE RECEIVED: %s" % pose)
            # for non-null w, we have r = v /  w
            self.assertAlmostEqual(pose['x'], 4.0/ math.pi , delta=0.15)
            self.assertAlmostEqual(pose['y'], -4.0/ math.pi , delta=0.15)
            self.assertAlmostEqual(pose['z'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['yaw'], -math.pi/2.0, delta=0.15)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=0.15)

########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(Spiral_Test)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

