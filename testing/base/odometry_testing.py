#! /usr/bin/env python
"""
This script tests some of the base functionalities of MORSE.
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

class OdometryTest(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        
        robot = Robot('atrv')
        robot.translate(x = 5.0, y = 2.0)
        robot.rotate(z = math.pi / 2)

        pose = Sensor('pose')
        robot.append(pose)
        pose.configure_mw('socket')

        motion = Actuator('v_omega')
        robot.append(motion)
        motion.configure_mw('socket')

        odo = Sensor('odometry')
        robot.append(odo)
        odo.configure_mw('socket')
        
        #env = Environment('indoors-1/indoor-1')
        env = Environment('land-1/rosace_1')
        env.configure_service('socket')

    def clear_datas(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw

    def record_datas(self, record):
        dx = record['dx']
        dy = record['dy']

        self.yaw += record['dyaw']
        
        # normalise angle
        while (self.yaw > math.pi):
            self.yaw+= -2 * math.pi
        while (self.yaw < -math.pi):
            self.yaw+= 2 * math.pi

        self.x += dx
        self.y += dy

    def odometry_test_helper(self, v, w, t):
        self.odo_stream.subscribe(self.record_datas)
        send_speed(self.v_w_client, v, w, t)
        self.odo_stream.unsubscribe(self.record_datas)

    
    def verify(self, expected_x, expected_y, expected_yaw):
        # Numerical integration is maybe not really good, so test with a
        # precision of 0.06
        precision = 0.06

        pose = self.pose_stream.get()
        odo = self.odo_stream.get()
        self.assertAlmostEqual(self.x, expected_x, delta=precision)
        self.assertAlmostEqual(self.y, expected_y, delta=precision)
        self.assertAlmostEqual(self.yaw, expected_yaw, delta=precision)
        self.assertAlmostEqual(pose['x'], 5.0 - expected_y, delta=precision)
        self.assertAlmostEqual(pose['y'], 2.0 + expected_x, delta=precision)
        self.assertAlmostEqual(pose['yaw'], expected_yaw + math.pi/2, delta=precision)
        self.assertAlmostEqual(odo['x'], expected_x, delta=precision)
        self.assertAlmostEqual(odo['y'], expected_y, delta=precision)
        self.assertAlmostEqual(odo['yaw'], expected_yaw, delta=precision)
        self.clear_datas(expected_x, expected_y, expected_yaw)

    def test_odometry(self):
        """ This test is guaranteed to be started only when the simulator
        is ready.
        """
        pass

        with Morse() as morse:
            # Read the start position, it must be (0.0, 0.0, 0.0)
            self.pose_stream = morse.stream('Pose')
            self.odo_stream = morse.stream('Odometry')

            # v_w socket
            port = morse.get_stream_port('Motion_Controller')
            self.v_w_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.v_w_client.connect(('localhost', port))
        
            self.clear_datas(0.0, 0.0, 0.0)

            self.odometry_test_helper(1.0, 0.0, 2.0)
            self.verify(2.0, 0.0, 0.0)

            self.odometry_test_helper(-1.0, 0.0, 2.0)
            self.verify(0.0, 0.0, 0.0)

            self.odometry_test_helper(1.0, -math.pi/4.0, 2.0)
            self.verify(4.0 / math.pi, -4.0/math.pi, -math.pi/2.0)

            self.odometry_test_helper(0.5, -math.pi/8.0, 12.0)
            self.verify(0.0, 0.0, 0.0)

            self.odometry_test_helper(-2.0, math.pi/2.0, 3.0)
            self.verify(4.0 / math.pi, -4.0/math.pi, -math.pi/2.0)


########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(OdometryTest)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

