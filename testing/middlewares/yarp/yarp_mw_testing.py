#! /usr/bin/env python
"""
This script tests some of the base functionalities of MORSE.
"""

import sys
import math
import subprocess
import yarp
from time import sleep
from morse.testing.testing import MorseTestCase
from pymorse import Morse

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

def send_speed_(s, v, w):
    b = s.prepare()
    b.clear()
    b.addDouble(v)
    b.addDouble(w)
    s.write(True)

def send_speed(s, v, w, t):
    send_speed_(s, v, w)
    sleep(t)
    send_speed_(s, 0.0, 0.0)

class YARP_MW_Test(MorseTestCase):

    def setUpMw(self):
        print("Launching yarpserver3")
        try:
            self.yarp_process = subprocess.Popen(['yarpserver3'])
        except OSError as ose:
            testlogger.error("Error while launching yarpserver3 ! Check you can run it from command-line\n")
            raise ose

    def tearDownMw(self):
        self.yarp_process.terminate()

    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        
        robot = Robot('atrv')

        pose = Sensor('pose')
        robot.append(pose)
        pose.configure_mw('yarp')

        motion = Actuator('v_omega')
        robot.append(motion)
        motion.configure_mw('yarp')
        
        env = Environment('indoors-1/indoor-1')
        env.configure_service('socket')

    def test_vw_controller(self):
        """ This test is guaranteed to be started only when the simulator
        is ready.
        """
        pass

        yarp.Network.init()

        pose_stream = yarp.BufferedPortBottle()
        pose_stream.open("/morse/test/pose/in")
        yarp.Network.connect("/morse/robots/ATRV/Pose/out", "/morse/test/pose/in")

        cmd_stream = yarp.BufferedPortBottle()
        cmd_stream.open("/morse/test/vw/out")
        yarp.Network.connect("/morse/test/vw/out", "/morse/robots/ATRV/Motion_Controller/in")
        
        # Read the start position, it must be (0.0, 0.0, 0.0)
        pose = pose_stream.read()
        for i in range(6):
            self.assertAlmostEqual(pose.get(i).asDouble(), 0.0, delta=0.1)


        send_speed(cmd_stream, 1.0, 0.0, 2.0)

        pose = pose_stream.read()
        self.assertAlmostEqual(pose.get(0).asDouble(), 2.0, delta=0.1)
        self.assertAlmostEqual(pose.get(1).asDouble(), 0.0, delta=0.1)
        self.assertAlmostEqual(pose.get(2).asDouble(), 0.0, delta=0.1)
        self.assertAlmostEqual(pose.get(3).asDouble(), 0.0, delta=0.1)
        self.assertAlmostEqual(pose.get(4).asDouble(), 0.0, delta=0.1)
        self.assertAlmostEqual(pose.get(5).asDouble(), 0.0, delta=0.1)

        send_speed(cmd_stream, -1.0, 0.0, 2.0)

        pose = pose_stream.read()
        for i in range(6):
            self.assertAlmostEqual(pose.get(i).asDouble(), 0.0, delta=0.1)

        send_speed(cmd_stream, 1.0, -math.pi/4.0, 2.0)
        pose = pose_stream.read()
        self.assertAlmostEqual(pose.get(0).asDouble(), 4.0 / math.pi, delta=0.1)
        self.assertAlmostEqual(pose.get(1).asDouble(), -4.0 / math.pi , delta=0.1)
        self.assertAlmostEqual(pose.get(2).asDouble(), 0.0, delta=0.1)
        self.assertAlmostEqual(pose.get(3).asDouble(), -math.pi/2.0, delta=0.1)
        self.assertAlmostEqual(pose.get(4).asDouble(), 0.0, delta=0.1)
        self.assertAlmostEqual(pose.get(5).asDouble(), 0.0, delta=0.1)

        send_speed(cmd_stream, 0.5, -math.pi/8.0, 12.0)
        pose = pose_stream.read()
        for i in range(6):
            self.assertAlmostEqual(pose.get(i).asDouble(), 0.0, delta=0.1)

        send_speed(cmd_stream, -2.0, math.pi/2.0, 3.0)
        pose = pose_stream.read()
        self.assertAlmostEqual(pose.get(0).asDouble(), 4.0 / math.pi, delta=0.1)
        self.assertAlmostEqual(pose.get(1).asDouble(), -4.0 / math.pi , delta=0.1)
        self.assertAlmostEqual(pose.get(2).asDouble(), 0.0, delta=0.1)
        self.assertAlmostEqual(pose.get(3).asDouble(), -math.pi/2.0, delta=0.1)
        self.assertAlmostEqual(pose.get(4).asDouble(), 0.0, delta=0.1)
        self.assertAlmostEqual(pose.get(5).asDouble(), 0.0, delta=0.1)

        yarp.Network.fini()

########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(YARP_MW_Test)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

