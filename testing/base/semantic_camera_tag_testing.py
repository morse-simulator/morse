#! /usr/bin/env python
"""
This script tests the Semantic Camera sensor and its 'tag' property in MORSE.
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

def send_dest(s, morse, x, y, yaw):
    s.publish({'x' : x, 'y' : y, 'z' : 0, \
                       'yaw' : yaw, 'pitch' : 0.0, 'roll' : 0.0})
    morse.sleep(0.5)

class Semantic_Camera_Test(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        robot = ATRV()
        camera = SemanticCamera()
        camera.properties(tag="Box")
        robot.append(camera)
        camera.translate(x=0.3, z=0.762)
        camera.add_stream('socket')

        motion = Teleport()
        robot.append(motion)
        motion.add_stream('socket')

        env = Environment('indoors-1/boxes')
        env.add_service('socket')
        box = bpymorse.get_object('RedBox')
        bpymorse.properties(box, Type="Box")

    def test_semantic_camera(self):
        """ This test is guaranteed to be started only when the simulator
        is ready.
        """
        with Morse() as morse:
            semantic_stream = morse.robot.camera
            teleport_client = morse.robot.motion

            o = semantic_stream.get()
            objects= o['visible_objects']
            self.assertEqual(objects, [])

            # Change the orientation of the robot using the v_w socket
            send_dest(teleport_client, morse, 0.0, 0.0, 5.0/4.0 * math.pi)

            # Second test for the sensor, with BlueBox in front
            o = semantic_stream.get()
            objects= o['visible_objects']
            self.assertEqual(objects, [])

            send_dest(teleport_client, morse, -5.0, 0.0, math.pi)

            # Third test for the sensor, with RedBox in front
            o = semantic_stream.get()
            objects= o['visible_objects']
            self.assertEqual(len(objects), 1)
            self.assertEqual(objects[0]['name'],'RedBox')
            self.assertAlmostEqual(objects[0]['position'][0], -7.48, delta=0.1)
            self.assertAlmostEqual(objects[0]['position'][1], 0.0, delta=0.1)

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(Semantic_Camera_Test)
