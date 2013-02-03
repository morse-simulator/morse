#! /usr/bin/env python
"""
This script tests the 'data stream' oriented feature of the socket interface.
"""

from morse.testing.testing import MorseTestCase

try:
    # Include this import to be able to use your test file as a regular 
    # builder script, ie, usable with: 'morse [run|exec] <your test>.py
    from morse.builder import *
except ImportError:
    pass

import sys
import time
from pymorse import Morse

def send_angles(orientation, yaw, pitch, roll):
    orientation.publish({'yaw' : yaw, 'pitch' : pitch, 'roll' : roll})

class CameraTest(MorseTestCase):

    def setUpEnv(self):
        
        atrv = ATRV()
        atrv.rotate(0.0, 0.0, math.pi)

        cam = VideoCamera('VideoCamera')
        cam.properties(capturing = True)
        cam.properties(cam_width = 320)
        cam.properties(cam_height = 240)
        cam.properties(cam_focal = 25.0000)
        cam.properties(Vertical_Flip = True)
        cam.translate(x=0.2, z=0.9)
        atrv.append(cam)
        cam.add_stream('socket')

        orientation = Orientation('orientation')
        orientation.add_stream('socket')
        atrv.append(orientation)

        env = Environment('indoors-1/boxes')

    def test_camera(self):
        """ Test if we can connect to the pose data stream, and read from it.
            
            The test has been constructed in the following way
                - put the robot in a specific position and takes an
                  image
                - using an editor image, look for the rgb value of some
                  specific zone, and setup some threshold
                - retrieve the threshold in the scene and tests it is
                  mostly invariant

            It will allow to detect that :
                - the camera is broken and don't have a real image
                - blender has changed the way they transmit the image
                - blender changes the orientation of their image
                - possibly other bugs
        """

        with Morse() as morse:
            cam_stream = morse.atrv.VideoCamera
            cam = cam_stream.get()

            orientation_stream = morse.atrv.orientation

            res = []
            # search for the red part in the image
            for i in range(320*240):
                o = cam['image'][i]
                # Value computed with gimp help ...
                if (o['r'] > 155 and o['g'] < 50 and o['b'] < 50):
                    res.append(i)

            # TODO redo those test
            self.assertTrue(len(res) > 140)
            for i in range(33742, 33747):
                self.assertTrue(i in res)
            for i in range(34064, 34096):
                self.assertTrue(i in res)
            for i in range(34064, 34096):
                self.assertTrue(i in res)
            for i in range(34387, 34416):
                self.assertTrue(i in res)
            # ...

            # Take random points outside the red block and check they're
            # not in this color plage
            for i in [0, 5000, 10000, 15000, 20000, 25000, 30000, 40000, 50000, 60000, 70000]:
                o = cam['image'][i]
                self.assertFalse(o['r'] > 150 and o['g'] < 50 and o['b'] < 50)

            send_angles(orientation_stream, 2.70, 0.0, 0)
            time.sleep(2.0)

            cam = cam_stream.get()
            res = []
            # search for the red block in the image
            for i in range(320*240):
                o = cam['image'][i]
                # Value computed with gimp help ...
                if (o['r'] > 155 and o['g'] < 50 and o['b'] < 50):
                    res.append(i)

            # TODO redo those test
            self.assertTrue(len(res) > 190)
            for i in range(32335, 32342):
                self.assertTrue(i in res)
            for i in range(32976, 32988):
                self.assertTrue(i in res)
            for i in range(33299, 33315):
                self.assertTrue(i in res)
            for i in range(33946, 33977):
                self.assertTrue(i in res)
            #  ...

            res = []
            # search the green block in the image
            for i in range(320*240):
                o = cam['image'][i]
                # Value computed with gimp help ...
                if (o['r'] < 5 and o['g'] > 110 and o['b'] < 5):
                    res.append(i)

            self.assertTrue(len(res) > 1000)
            # the green block is better defined
            for i in range(218, 237):
                for j in range(97, 145):
                    self.assertTrue( (j * 320 + i) in res)


            # take random point and check they are not in the green or
            # red block
            for i in [0, 5000, 10000, 15000, 20000, 25000, 30000, 40000, 50000, 60000, 70000]:
                o = cam['image'][i]
                self.assertFalse(o['r'] > 150 and o['g'] < 50 and o['b'] < 50)
                self.assertFalse(o['r'] < 5 and o['g'] > 110 and o['b'] < 5)



########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(CameraTest)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

