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
import base64
import struct
import zlib
from pymorse import Morse

IMAGE_WIDTH=320
IMAGE_HEIGHT=240
DEBUG_PGM=False # turn this to True to save difference images

def send_angles(orientation_stream, yaw, pitch, roll):
    orientation_stream.publish({'yaw' : yaw, 'pitch' : pitch, 'roll' : roll})

# a,b in radians.  d in digits of accuracy (ugh!) on the unit circle.
def are_angles_almost_equal(a, b, delta):
    return abs(math.cos(a) - math.cos(b)) <= delta and \
           abs(math.sin(a) - math.sin(b)) <= delta

def wait_yaw(gyroscope_stream, morse, timeout, yaw, precision):
    print("time::now %f" % morse.time())
    timeout_t = morse.time() + timeout
    print("wait_yaw %f %f" % (morse.time(), timeout_t))
    while morse.time() < timeout_t:
        print("%f %f" % (morse.time(), timeout_t))
        angles = gyroscope_stream.get()
        if are_angles_almost_equal(angles['yaw'], yaw, precision):
            return True
        morse.sleep(.1)

    return False

def rotate_robot_and_wait(orientation_stream, gyroscope_stream, morse, \
                          yaw, timeout, precision):
    send_angles(orientation_stream, yaw, 0.0, 0.0)
    # wait for the robot to be at the desired Z angle (yaw)
    return wait_yaw(gyroscope_stream, morse, timeout, yaw, precision)

def flush_camera(camera_stream, morse, timeout):
    timeout_t = morse.time() + timeout
    print("flush_camera %f" % timeout_t)
    while morse.time() < timeout_t:
        print("%f %f" % (morse.time(), timeout_t))
        # get a new image from the camera stream
        camera_stream.get()
        morse.sleep(.1)

def rgba2gray8u(image_rgba_base64):
    # Grayscale model used for HDTV developed by the ATSC (Wikipedia)
    image = base64.b64decode( image_rgba_base64 )
    return [ int(0.2126 * image[index] +
                 0.7152 * image[index + 1] +
                 0.0722 * image[index + 2] )
             for index in range(0, len(image), 4) ]

def load_image(filepath, size):
    image = []
    with open(filepath, 'rb') as f:
        # Read binary image (Bytes)
        image = struct.unpack('%iB'%size, zlib.decompress(f.read()))

    return image

def save_image(filepath, image8u):
    with open(filepath, 'wb') as f:
        # Write binary image (Bytes)
        f.write(zlib.compress(struct.pack('%iB'%len(image8u), *image8u)))

def save_pgm_ascii(filepath, image8u, width, height):
    assert(len(image8u) == width * height)
    with open(filepath, 'w') as f:
        # Write PGM P2 image (ASCII)
        f.write('P2\n%i %i\n255\n'%(width, height))
        f.write(' '.join(['%i'%px for px in image8u]))

# http://stackoverflow.com/questions/7368739/numpy-and-16-bit-pgm
def read_pgm_ascii(filepath):
    with open(filepath, 'r') as f:
        buff = f.read()
    try:
        import re
        header, width, height, maxval = re.search(
            "(^P2\s(?:\s*#.*[\r\n])*"
            "(\d+)\s(?:\s*#.*[\r\n])*"
            "(\d+)\s(?:\s*#.*[\r\n])*"
            "(\d+)\s(?:\s*#.*[\r\n]\s)*)", buff).groups()
    except AttributeError:
        raise ValueError("Not PGM P2 file: '%s'" % filepath)
    image8u = [int(px) for px in buff[len(header):].split()]
    assert(len(image8u) == int(width) * int(height))
    assert(max(image8u) <= int(maxval))
    return image8u

def capture8u(cam_stream, image_path=None):
    # get new RGBA image
    capture = cam_stream.get()
    # convert it to grayscale
    image8u = rgba2gray8u( capture['image'] )
    # or if we use Video8uPublisher
    # image8u = base64.b64decode( capture['image'] )
    # save the image (for debug)
    if image_path:
        save_pgm_ascii(image_path, image8u, IMAGE_WIDTH, IMAGE_HEIGHT)

    return image8u

def diff_image(imageA, imageB, debug=None):
    assert(len(imageA) == len(imageB))
    diff = [abs(pxA - pxB) for pxA, pxB in zip(imageA, imageB)]
    # returns sum(abs( imageA - imageB )) pixel per pixel
    sum_diff = sum(diff)
    if debug:
        pgm_path = '%s.%i.%s.pgm'%(os.path.abspath(__file__), \
                                   int(time.time()*1000), debug)
        save_pgm_ascii(pgm_path, diff, IMAGE_WIDTH, IMAGE_HEIGHT)
        print("debug: %8i -> %5.3f %% %s"% \
              (sum_diff, sum_diff/(len(diff)*2.55), pgm_path))

    return sum_diff

def normalize_radians(angle):
    two_pi = 2 * math.pi
    # First, normalize angle between 0 and 2*PI
    new_angle = angle % two_pi
    # Then, between -PI and PI
    if new_angle > math.pi:
        new_angle = new_angle - two_pi

    return new_angle

class CameraTest(MorseTestCase):

    def setUpEnv(self):

        atrv = ATRV('atrv')
        atrv.rotate(0.0, 0.0, math.pi)

        camera = VideoCamera('camera')
        camera.properties(capturing = True)
        camera.properties(cam_width = 320)
        camera.properties(cam_height = 240)
        camera.properties(cam_focal = 25.0000)
        camera.properties(Vertical_Flip = True)
        camera.translate(x=0.2, z=0.9)
        atrv.append(camera)
        camera.add_stream('socket')

        orientation = Orientation('orientation')
        orientation.add_stream('socket')
        atrv.append(orientation)

        gyroscope = Gyroscope('gyroscope')
        gyroscope.add_stream('socket')
        atrv.append(gyroscope)

        env = Environment('indoors-1/boxes')
        # Shadow may vary depending on the GPU, MULTITEXTURE mode = no shadow.
        # We can't use SOLID viewport or SINGLETEXTURE mode since they do not
        # provide image in bge.texture
        env.set_material_mode('MULTITEXTURE')
        camera.profile()

    def assert_image_file_diff_less(self, filepath, image8u, delta):
        image_from_file = read_pgm_ascii(filepath)
        self.assert_images_diff_less(image8u, image_from_file, delta)

    def assert_images_diff_less(self, imageA, imageB, delta):
        debug = "less" if DEBUG_PGM else None
        diff = diff_image(imageA, imageB, debug=debug)
        # Diff max is: width * height * 255
        # delta in percent (of the maximum difference)
        self.assertLess(diff, delta * 2.55 * len(imageA))

    def assert_images_diff_greater(self, imageA, imageB, delta):
        debug = "greater" if DEBUG_PGM else None
        diff = diff_image(imageA, imageB, debug=debug)
        # Diff max is: width * height * 255
        # delta in percent (of the maximum difference)
        self.assertGreater(diff, delta * 2.55 * len(imageA))

    def assert_orientation(self, gyroscope_stream, yaw, pitch, roll, precision):
        angles = gyroscope_stream.get()
        self.assertAnglesAlmostEqual(angles['yaw'], yaw, precision)
        self.assertAnglesAlmostEqual(angles['pitch'], pitch, precision)
        self.assertAnglesAlmostEqual(angles['roll'], roll, precision)

    # a,b in radians.  d in digits of accuracy (ugh!) on the unit circle.
    # http://astrometry.net/svn/trunk/projects/ephemeris/py/test_celestial_mechanics.py
    def assertAnglesAlmostEqual(self, a, b, delta=0.0):
        self.assertAlmostEqual(math.cos(a), math.cos(b), delta=delta)
        self.assertAlmostEqual(math.sin(a), math.sin(b), delta=delta)

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
            camera_stream = morse.atrv.camera
            orientation_stream = morse.atrv.orientation
            gyroscope_stream = morse.atrv.gyroscope

            imageA_path = '%s.A.pgm'%os.path.abspath(__file__)
            imageB_path = '%s.B.pgm'%os.path.abspath(__file__)

            precision = 0.01
            # assert robot orienation is correct
            self.assert_orientation(gyroscope_stream, math.pi, 0.0, 0.0, \
                                    precision)

            # get a new image from the camera in gray
            imageA = capture8u(camera_stream)#, imageA_path)
            # assert that the camera image differ < .1 percent from the expected
            self.assert_image_file_diff_less(imageA_path, imageA, 0.1)

            # command the robot to rotate and wait that he does for 5 seconds max
            in_time = rotate_robot_and_wait(orientation_stream, \
                                            gyroscope_stream, morse, 2.70, 5, precision)
            # XXX robot might have not graphically turned yet! happens randomly!
            # gyroscope can give its new orientation while the physics didnt update yet.
            if DEBUG_PGM:
                print("debug: rotate in time: %s (False = timeout)"%str(in_time))
            # "flush" the camera stream for 1 second
            flush_camera(camera_stream, morse, 1.0)

            # assert robot orienation is correct
            self.assert_orientation(gyroscope_stream, 2.70, 0.0, 0.0, precision)

            # get a new image from the camera in gray
            imageB = capture8u(camera_stream)#, imageB_path)
            # assert that the camera image differ < .1 percent from the expected
            self.assert_image_file_diff_less(imageB_path, imageB, 0.2)

            # assert that the second image differ > 4 percent from the first
            self.assert_images_diff_greater(imageA, imageB, 4)

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(CameraTest)
