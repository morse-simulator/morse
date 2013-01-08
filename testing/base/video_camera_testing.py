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

import os
import sys
import socket
import json
import time
from pymorse import Morse

def send_angles(s, yaw, pitch, roll):
    s.send(json.dumps({'yaw' : yaw, 'pitch' : pitch, 'roll' : roll}).encode())

class CameraTest(MorseTestCase):

    def setUpEnv(self):
        
        atrv = ATRV()
        atrv.rotate(0.0, 0.0, math.pi)

        cam = Sensor('video_camera') # TODO bug pose relative (ATRV Z=+0.1)
        #cam = CameraVideo() # TODO bug pymorse socket port ?
        # "pymorse.py", line 242, in run: _client.connect((self.host, self.port))
        # OverflowError: getsockaddrarg: port must be 0-65535.
        cam.properties(capturing = True)
        cam.properties(cam_width = 320)
        cam.properties(cam_height = 240)
        cam.properties(cam_focal = 25.0000)
        cam.properties(Vertical_Flip = True)
        cam.translate(x=0.2, z=0.9)
        atrv.append(cam)
        cam.configure_mw('socket')

        orientation = Actuator('orientation')
        orientation.configure_mw('socket')
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
            cam_stream = morse.stream('CameraMain')

            cam = cam_stream.get()

            port = morse.get_stream_port('Motion_Controller')
            orientation_stream = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            orientation_stream.connect(('localhost', port))

            res = []
            # search for the red part in the image
            for i in range(320*240):
                o = cam['image'][i]
                # Value computed with gimp help ...
                if (o['r'] > 155 and o['g'] < 50 and o['b'] < 50):
                    res.append(i)

            self.assertTrue(len(res) > 140)
            self.assertTrue(32000 < min(res) < 34000)
            self.assertTrue(35000 < max(res) < 37000)

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

            self.assertTrue(len(res) > 140)
            self.assertTrue(31000 < min(res) < 34000)
            self.assertTrue(34000 < max(res) < 36000)

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

