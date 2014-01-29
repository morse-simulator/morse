""" ROS-based MorseTestCase (unit-tests)

The main feature is to launch ROS core at start.
"""

from morse.testing.testing import MorseTestCase, testlogger

import os
import sys
import subprocess

try:
    import roslib
except ImportError as error:
    testlogger.error("Could not find ROS. source setup.[ba]sh ?")
    raise error

class RosTestCase(MorseTestCase):
    def setUpMw(self):
        try:
            self.roscore_process = subprocess.Popen(['roscore'])
        except OSError as ose:
            testlogger.error("Error while launching roscore ! Check you can run it from command-line\n")
            raise ose

    def tearDownMw(self):
        self.roscore_process.terminate()
