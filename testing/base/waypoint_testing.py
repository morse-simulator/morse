#! /usr/bin/env python
"""
This script tests the waypoint actuator paired with the odometry sensor
in MORSE.
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
    from morse.builder.morsebuilder import *
except ImportError:
    pass

def send_destination(s, x, y, z, t, tolerance=0.4, speed=4.0):
    s.send(json.dumps({'x': x, 'y': y, 'z': z, 'tolerance': tolerance, 'speed': speed}).encode())
    sleep(t)


class Waypoint_Test(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        
        robot = Robot('atrv')

        pose = Sensor('pose')
        robot.append(pose)
        pose.configure_mw('socket')

        motion = Actuator('waypoint')
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

        self.yaw+= record['dyaw']
        
        # normalise angle
        while (self.yaw > math.pi): 
            self.yaw+= -2 * math.pi
        while (self.yaw < -math.pi):
            self.yaw+= 2 * math.pi

#        self.x+= dx * math.cos(self.yaw)
#        self.y+= dx * math.sin(self.yaw)
        self.x += dx
        self.y += dy

    def odometry_test_helper(self, x, y, z, t, tolerance=0.4, speed=4.0):
        pose = self.pose_stream.get()
        self.clear_datas(pose['x'], pose['y'], pose['yaw'])
        self.odo_stream.subscribe(self.record_datas)
        send_destination(self.waypoint_client, x, y, z, t, tolerance, speed)
        self.odo_stream.unsubscribe(self.record_datas)


    def test_odometry(self):
        """ This test is guaranteed to be started only when the simulator
        is ready.
        """
        pass

        morse = Morse()
        
        # Read the start position, it must be (0.0, 0.0, 0.0)
        self.pose_stream = morse.stream('Pose')
        self.odo_stream = morse.stream('Odometry')

        # v_w socket
        port = morse.get_stream_port('Motion_Controller')
        self.waypoint_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.waypoint_client.connect(('localhost', port))

        # Numerical integration is maybe not really good, so test with a
        # precision of 0.05
        self.odometry_test_helper(2.0, 4.0, 0.0, 5.0)
        self.assertAlmostEqual(self.x, 2.0, delta=0.55)
        self.assertAlmostEqual(self.y, 4.0, delta=0.55)
        #self.assertAlmostEqual(self.yaw, 0.0, delta=0.05)

        self.odometry_test_helper(-3.0, -2.0, 0.0, 5.0)
        self.assertAlmostEqual(self.x, -3.0, delta=0.55)
        self.assertAlmostEqual(self.y, -2.0, delta=0.55)
        #self.assertAlmostEqual(self.yaw, 0.0, delta=0.05)

        self.odometry_test_helper(0.0, 0.0, 0.0, 5.0)
        self.assertAlmostEqual(self.x, 0.0, delta=0.55)
        self.assertAlmostEqual(self.y, 0.0, delta=0.55)
        #self.assertAlmostEqual(self.yaw, -math.pi/2.0, delta=0.05)

        morse.close()

########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(Waypoint_Test)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

