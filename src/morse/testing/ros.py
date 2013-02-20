""" This module defines a specialization of MorseTestCase for ROS-based
unit-tests.

Its main feature is to launch ROS core at start, and to add the special 
ROS node 'morsetesting' to the ROS_PACKAGE_PATH env variable.

Attention! Because this module may manipulate the ROS_PACKAGE_PATH, it 
*must* be imported in unit-test before importing roslib!
"""

from morse.testing.testing import MorseTestCase, testlogger

import logging
roslogger = logging.getLogger("morsetesting.ros")
roslogger.setLevel(logging.DEBUG)
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
formatter = logging.Formatter('[%(asctime)s (%(levelname)s)]   %(message)s')
ch.setFormatter(formatter)
roslogger.addHandler(ch)

class RosInstallationError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

import os
import sys
import subprocess

try:
    os.environ['MORSE_SRC_ROOT']
except KeyError:
    raise RosInstallationError("You must define the environment "
        "variable MORSE_SRC_ROOT to point to the MORSE source "
        "before running ROS tests.")

os.environ['ROS_PACKAGE_PATH'] += ":" + os.path.join(os.environ['MORSE_SRC_ROOT'], "testing", "middlewares", "ros")
os.environ['ROS_PACKAGE_PATH'] += ":" + os.path.join(os.environ['MORSE_SRC_ROOT'], "testing", "robots", "pr2")


try:
    import roslib
    from rospkg.common import ResourceNotFound
except ImportError:
    raise RosInstallationError("Can not import roslib. ROS is not installed?")

class RosTestCase(MorseTestCase):
    def setUpMw(self):
        try:
            self.roscore_process = subprocess.Popen(['roscore'])
        except OSError as ose:
            testlogger.error("Error while launching roscore ! Check you can run it from command-line\n")
            raise ose

    def tearDownMw(self):
        self.roscore_process.terminate()


