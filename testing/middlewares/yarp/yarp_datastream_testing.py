#! /usr/bin/env python
"""
This script tests some of the base functionalities of MORSE.
"""

import sys
import math
import subprocess
import yarp
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

def send_speed(s, morse, v, w, t):
    send_speed_(s, v, w)
    morse.sleep(t)
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
        
        robot = ATRV()

        pose = Pose()
        pose.translate(z=-0.10)
        robot.append(pose)
        pose.add_stream('yarp')

        motion = MotionVW()
        robot.append(motion)
        motion.add_stream('yarp')
        
        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def test_vw_controller(self):
        """ This test is guaranteed to be started only when the simulator
        is ready.
        """
        pass

        yarp.Network.init()

        pose_stream = yarp.BufferedPortBottle()
        pose_stream.open("/morse/test/pose/in")
        yarp.Network.connect("/morse/robot/pose/out", "/morse/test/pose/in")

        cmd_stream = yarp.BufferedPortBottle()
        cmd_stream.open("/morse/test/vw/out")
        yarp.Network.connect("/morse/test/vw/out", "/morse/robot/motion/in")
        
        with Morse() as morse:
            # Read the start position, it must be (0.0, 0.0, 0.0)

            precision = 0.20
            pose = pose_stream.read()
            for i in range(1, 7):
                self.assertAlmostEqual(pose.get(i).asDouble(), 0.0, delta = precision)


            send_speed(cmd_stream, morse, 1.0, 0.0, 2.0)

            pose = pose_stream.read()
            self.assertAlmostEqual(pose.get(1).asDouble(), 2.0, delta = precision)
            self.assertAlmostEqual(pose.get(2).asDouble(), 0.0, delta = precision)
            self.assertAlmostEqual(pose.get(3).asDouble(), 0.0, delta = precision)
            self.assertAlmostEqual(pose.get(4).asDouble(), 0.0, delta = precision)
            self.assertAlmostEqual(pose.get(5).asDouble(), 0.0, delta = precision)
            self.assertAlmostEqual(pose.get(6).asDouble(), 0.0, delta = precision)

            send_speed(cmd_stream, morse, -1.0, 0.0, 2.0)

            pose = pose_stream.read()
            for i in range(1, 7):
                self.assertAlmostEqual(pose.get(i).asDouble(), 0.0, delta = precision)

            send_speed(cmd_stream, morse, 1.0, -math.pi/4.0, 2.0)
            pose = pose_stream.read()
            self.assertAlmostEqual(pose.get(1).asDouble(), 4.0 / math.pi, delta = precision)
            self.assertAlmostEqual(pose.get(2).asDouble(), -4.0 / math.pi , delta = precision)
            self.assertAlmostEqual(pose.get(3).asDouble(), 0.0, delta = precision)
            self.assertAlmostEqual(pose.get(4).asDouble(), -math.pi/2.0, delta = precision)
            self.assertAlmostEqual(pose.get(5).asDouble(), 0.0, delta = precision)
            self.assertAlmostEqual(pose.get(6).asDouble(), 0.0, delta = precision)

            send_speed(cmd_stream, morse, 0.5, -math.pi/8.0, 12.0)
            pose = pose_stream.read()
            for i in range(1, 7):
                self.assertAlmostEqual(pose.get(i).asDouble(), 0.0, delta = precision)

            send_speed(cmd_stream, morse, -2.0, math.pi/2.0, 3.0)
            pose = pose_stream.read()
            self.assertAlmostEqual(pose.get(1).asDouble(), 4.0 / math.pi, delta = precision)
            self.assertAlmostEqual(pose.get(2).asDouble(), -4.0 / math.pi , delta = precision)
            self.assertAlmostEqual(pose.get(3).asDouble(), 0.0, delta = precision)
            self.assertAlmostEqual(pose.get(4).asDouble(), -math.pi/2.0, delta = precision)
            self.assertAlmostEqual(pose.get(5).asDouble(), 0.0, delta = precision)
            self.assertAlmostEqual(pose.get(6).asDouble(), 0.0, delta = precision)

        yarp.Network.fini()

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(YARP_MW_Test)
