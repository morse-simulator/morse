#! /usr/bin/env python
"""
This script tests the 'data stream' oriented feature of the socket interface.
"""

from morse.testing.testing import MorseTestCase

try:
    # Include this import to be able to use your test file as a regular 
    # builder script, ie, usable with: 'morse [run|exec] <your test>.py
    from morse.builder import *
    from morse.builder.actuators import Light
except ImportError:
    pass

import sys
import time
from pymorse import Morse

class LightTest(MorseTestCase):

    def setUpEnv(self):
        atrv = ATRV()

        cam = VideoCamera()
        cam.properties(capturing = True, cam_width = 320, cam_height = 240, \
                       cam_focal = 25.0000, Vertical_Flip = True)
        cam.translate(x=0.2, z=0.9)
        atrv.append(cam)
        cam.add_stream('socket')

        light = Light('light')
        light.translate(x=0.2, z=1.1)
        light.properties(Distance = 50.0)
        atrv.append(light)
        light.add_stream('socket')

        block  = PassiveObject('environments/indoors-1/boxes', 'GreenBox')
        block.translate(x=2, y=0, z=1)

        env = Environment('empty')

    def test_light(self):
        with Morse() as morse:
            cam_stream = morse.atrv.cam
            light_stream = morse.atrv.light

            light_stream.publish({"emit": False})

            time.sleep(1.0)

            # Light is shutdown. There is no light source on the scene,
            # so camera can't distinguish color
            res = []

            # search the green block in the image
            cam = cam_stream.get()
            for i in range(320*240):
                o = cam['image'][i]
                # Value computed with gimp help ...
                if (o['r'] < 5 and o['g'] > 110 and o['b'] < 5):
                    res.append(i)

            self.assertEqual(len(res), 0)

            # Now, illuminate the scene
            light_stream.publish({"emit": True})

            time.sleep(2.0)
            cam = cam_stream.get()
            # search the green block in the image
            for i in range(320*240):
                o = cam['image'][i]
                # Value computed with gimp help ...
                if (o['r'] < 5 and o['g'] > 110 and o['b'] < 5):
                    res.append(i)

            self.assertTrue(len(res) > 10000)


########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(LightTest)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

