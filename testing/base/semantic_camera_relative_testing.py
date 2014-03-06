#! /usr/bin/env python
"""
This script tests the Semantic Camera sensor and its 'relative' property in MORSE.
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
        camera.properties(relative=True)
        robot.append(camera)
        camera.translate(x=0.3, z=0.762)
        #camera.rotate(x=math.pi/2.0)
        camera.add_stream('socket')

        motion = Teleport()
        robot.append(motion)
        motion.add_stream('socket')

        env = Environment('indoors-1/boxes')
        env.add_service('socket')


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
            send_dest(teleport_client, morse, -5.0, 0.0, math.pi)

            # Second test for the sensor, with objects in front
            o = semantic_stream.get()
            objects= o['visible_objects']
            self.assertEqual(len(objects), 1)
            self.assertEqual(objects[0]['name'],'RedBox')
            # RedBox in camera frame:
            position = [0, -0.2, 2.2]
            quaternion = {'x':0.5, 'y':0.5, 'z':-0.5, 'w':0.5}
            # quaternion is equal to euler (pi, pi, 0) in XYZ mode
            for i in [0,1,2]:
                self.assertAlmostEqual(objects[0]['position'][i], position[i], delta=0.1)
            for i in ['x', 'y', 'z', 'w']:
                self.assertAlmostEqual(objects[0]['orientation'][i], quaternion[i], delta=.1)

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(Semantic_Camera_Test)
