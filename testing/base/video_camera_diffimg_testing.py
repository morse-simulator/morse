#! /usr/bin/env python
# -*- coding: utf-8 -*-
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
import time
import struct
import zlib
from pymorse import Morse

IMAGE_WIDTH=320
IMAGE_HEIGHT=240
DEBUG_PGM=True

def send_angles(s, yaw, pitch, roll):
    s.publish({'yaw' : yaw, 'pitch' : pitch, 'roll' : roll})

class CameraTest(MorseTestCase):

    def setUpEnv(self):

        atrv = ATRV()
        atrv.rotate(0.0, 0.0, math.pi)

        cam = Sensor('video_camera')
        cam.name = 'VideoCamera'
        #cam = VideoCamera('VideoCamera') # TODO bug look +Z
        cam.properties(capturing = True)
        cam.properties(cam_width = 320)
        cam.properties(cam_height = 240)
        cam.properties(cam_focal = 25.0000)
        cam.properties(Vertical_Flip = True)
        cam.translate(x=0.2, z=0.9)
        atrv.append(cam)
        cam.configure_mw('socket')

        orientation = Orientation('orientation')
        orientation.configure_mw('socket')
        atrv.append(orientation)

        env = Environment('indoors-1/boxes')

    def rgb2gray8u(self, image_rgb_data):
        # Grayscale model used for HDTV developed by the ATSC (Wikipedia)
        return [int(0.2126 * px['r'] + 0.7152 * px['g'] + 0.0722 * px['b']) \
                for px in image_rgb_data ]

    def diff_image(self, imageA, imageB, debug=False):
        diff = [abs(pxA - pxB) for pxA, pxB in zip(imageA, imageB)]
        # returns sum(abs( imageA - imageB )) pixel per pixel
        sum_diff = sum(diff)
        if debug:
            pgm_path = '%s.%i.pgm'%(os.path.abspath(__file__), int(time.time()*1000))
            self.save_pgm(pgm_path, diff, IMAGE_WIDTH, IMAGE_HEIGHT)
            print("%i :: %.3f %% :: %s"%(sum_diff, sum_diff*100./(320*240*255), pgm_path))
        return sum_diff

    def save_image(self, filepath, image8u):
        with open(filepath, 'wb') as f:
            # Write binary image (Bytes)
            f.write(zlib.compress(struct.pack('%iB'%len(image8u), *image8u)))

    def save_pgm(self, filepath, image8u, width, height):
        assert(len(image8u) == height * width)
        with open(filepath, 'w') as f:
            # Write binary image (Bytes)
            f.write('P2\n%i %i\n255\n'%(width, height) + \
                    '\n'.join(['%i'%px for px in image8u]))

    def load_image(self, filepath, size):
        image = []
        with open(filepath, 'rb') as f:
            # Read binary image (Bytes)
            image = struct.unpack('%iB'%size, zlib.decompress(f.read()))
        return image

    def assert_image_diff_less(self, filepath, image8u, delta):
        # delta in percent (of the maximum difference)
        diff = self.get_diff_file_image(filepath, image8u)
        # Diff max is: width * height * 255
        self.assertLess(diff, delta * 2.55 * len(image8u))

    def assert_image_diff_greater(self, imageA, imageB, delta):
        # delta in percent (of the maximum difference)
        diff = self.diff_image(imageA, imageB, \
                               debug=DEBUG_PGM)
        # Diff max is: width * height * 255
        self.assertGreater(diff, delta * 2.55 * len(imageA))

    def get_diff_file_image(self, filepath, image8u):
        # Diff max is: width * height * 255
        return self.diff_image(image8u, self.load_image(filepath, len(image8u)),\
                               debug=DEBUG_PGM)

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
            cam_stream = morse.robot.VideoCamera
            orientation_stream = morse.robot.orientation

            test_path = os.path.dirname(os.path.abspath(__file__))
            imageA_path = os.path.join(test_path, 'video_camera_diffimgA.data')
            imageB_path = os.path.join(test_path, 'video_camera_diffimgB.data')

            cam = cam_stream.get() # get new image
            imageA = self.rgb2gray8u(cam['image']) # grayscale
            #self.save_image(imageA_path, imageA) # save the image
            # delta is choosen as 0.05 percent
            # assert that the camera image differ < .5 percent from the expected
            self.assert_image_diff_less(imageA_path, imageA, 0.5)

            send_angles(orientation_stream, 2.70, 0.0, 0)
            time.sleep(2.0)

            cam = cam_stream.get() # get new image
            imageB = self.rgb2gray8u(cam['image']) # grayscale
            #self.save_image(imageB_path, imageB) # save the image
            # delta is choosen as 0.1 percent (physics engine tilt)
            # assert that the camera image differ < 2 percent from the expected
            self.assert_image_diff_less(imageB_path, imageB, 2)

            # assert that the second image differ > 5 percent from the first
            self.assert_image_diff_greater(imageA, imageB, 5)

########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(CameraTest)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

