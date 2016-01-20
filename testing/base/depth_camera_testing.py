#! /usr/bin/env python
"""
This script tests the Depth camera in MORSE.
"""

import sys
import math
import struct
import base64
from morse.testing.testing import MorseTestCase
from pymorse import Morse

# Include this import to be able to use your test file as a regular
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

class DepthCameraTest(MorseTestCase):

    def setUpEnv(self):
        """ Defines the test scenario """

        robot = ATRV()

        motion = MotionVW()
        robot.append(motion)
        motion.add_stream('socket')

        camera = DepthCamera()
        camera.translate(z = 1)
        camera.frequency(3)
        robot.append(camera)
        camera.add_stream('socket')

        env = Environment('indoors-1/boxes')
        # No fastmode here, no MaterialIndex in WIREFRAME mode: AttributeError:
        # 'KX_PolygonMaterial' object has no attribute 'getMaterialIndex'

    def test_depth_camera(self):
        """ Assert that for every points : near <= z <= far """

        with Morse() as morse:
            # turn around
            morse.robot.motion.publish({'v': 1, 'w': 1})

            for step in range(5):
                msg  = morse.robot.camera.get()
                data = base64.b64decode( msg['points'] )

                # assert that : near <= z <= far
                for i in range(0, len(data) - 12, 12):
                    xyz = struct.unpack('fff', data[i:i+12])
                    self.assertGreaterEqual(xyz[2], 1)
                    self.assertLessEqual(xyz[2], 20)

                morse.sleep(0.2) # wait for turning

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(DepthCameraTest)
