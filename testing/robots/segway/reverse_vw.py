#! /usr/bin/env python
"""
This script tests the Segway RMP400 robot with differential drive actuator
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
    s.rpc('MotionVWDiff', 'set_speed', v, w)
    sleep(t)
    s.rpc('MotionVWDiff', 'stop')
    sleep(1)

class Differential_VW_Test(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        
        robot = SegwayRMP400()
        robot.translate(z=0.1)
        robot.unparent_wheels()

        pose = Pose('Pose')
        robot.append(pose)
        pose.add_stream('socket')
        pose.add_stream('text')

        motion = MotionVWDiff('MotionVWDiff')
        robot.append(motion)
        motion.add_stream('socket')
        motion.add_service('socket')
        
        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def test_vw_controller(self):
        with Morse() as morse:
        
            # Read the start position, it must be (0.0, 0.0, 0.0)
            pose_stream = morse.stream('Pose')
            pose = pose_stream.get()
            for key,coord in pose.items():
                if key == 'z':
                    self.assertAlmostEqual(coord, 0.20, delta=0.03)
                else:
                    self.assertAlmostEqual(coord, 0.0, delta=0.03)

            # v_w socket
            port = morse.get_stream_port('MotionVWDiff')
            v_w_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            v_w_client.connect(('localhost', port))

            send_speed(v_w_client, -1.0, 0.0, 2.0)

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], -2.0, delta=0.15)
            self.assertAlmostEqual(pose['y'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['z'], 0.2, delta=0.15)
            self.assertAlmostEqual(pose['yaw'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=0.15)

            send_speed(v_w_client, 1.0, 0.0, 2.0)

            pose = pose_stream.get()
            for key,coord in pose.items():
                if key == 'z':
                    self.assertAlmostEqual(coord, 0.20, delta=0.15)
                else:
                    self.assertAlmostEqual(coord, 0.0, delta=0.15)

            """
            send_speed(v_w_client, 0.0, -math.pi/4.0, 2.0)

            pose = pose_stream.get()
            # for non-null w, we have r = v /  w
            self.assertAlmostEqual(pose['x'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['y'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['z'], 0.10, delta=0.15)
            self.assertAlmostEqual(pose['yaw'], -math.pi/2.0, delta=0.15)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=0.15)

            send_speed(v_w_client, 0.0, math.pi/4.0, 2.0)

            pose = pose_stream.get()
            for key,coord in pose.items():
                if key == 'z':
                    self.assertAlmostEqual(coord, 0.10, delta=0.15)
                else:
                    self.assertAlmostEqual(coord, 0.0, delta=0.15)
            """


            send_speed(v_w_client, -1.0, math.pi/4.0, 2.0)

            pose = pose_stream.get()
            # for non-null w, we have r = v /  w
            self.assertAlmostEqual(pose['x'], -4.0/ math.pi , delta=0.20)
            self.assertAlmostEqual(pose['y'], -4.0/ math.pi , delta=0.20)
            self.assertAlmostEqual(pose['z'], 0.2, delta=0.20)
            self.assertAlmostEqual(pose['yaw'], math.pi/2.0, delta=0.20)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=0.20)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=0.20)

            send_speed(v_w_client, -0.5, math.pi/8.0, 12.0)

            pose = pose_stream.get()
            for key,coord in pose.items():
                if key == 'z':
                    self.assertAlmostEqual(coord, 0.20, delta=0.15)
                else:
                    self.assertAlmostEqual(coord, 0.0, delta=0.20)

            send_speed(v_w_client, 2.0, -math.pi/2.0, 3.0)

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], -4.0/ math.pi , delta=0.20)
            self.assertAlmostEqual(pose['y'], -4.0/ math.pi , delta=0.20)
            self.assertAlmostEqual(pose['z'], 0.2, delta=0.20)
            self.assertAlmostEqual(pose['yaw'], -math.pi/2.0, delta=0.20)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=0.20)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=0.20)

    def X_test_vw_service_controller(self):
    #def test_vw_service_controller(self):
        with Morse() as morse:
        
            # Read the start position, it must be (0.0, 0.0, 0.0)
            pose_stream = morse.stream('Pose')
            pose = pose_stream.get()
            for key,coord in pose.items():
                if key == 'z':
                    self.assertAlmostEqual(coord, 0.20, delta=0.02)
                else:
                    self.assertAlmostEqual(coord, 0.0, delta=0.02)

            send_service_speed(morse, 1.0, 0.0, 2.0)

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 2.0, delta=0.15)
            self.assertAlmostEqual(pose['y'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['z'], 0.2, delta=0.15)
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
            self.assertAlmostEqual(pose['z'], 0.2, delta=0.15)
            self.assertAlmostEqual(pose['yaw'], -math.pi/2.0, delta=0.15)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=0.15)

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(Differential_VW_Test)
