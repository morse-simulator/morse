#! /usr/bin/env python
"""
This script tests some of the base functionalities of MORSE.
"""

import sys
from morse.testing.testing import MorseTestCase

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

class BaseTest(MorseTestCase):

    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        r2d2 = Robot('res/r2d2.urdf', name='r2d2')
        env = Environment('empty', fastmode = True)

    def test_urdf(self):
        """ Tests the simulator can return the list of robots
        
        This test is guaranteed to be started only when the simulator
        is ready.
        """
        with Morse() as morse:
            r2d2 = morse.r2d2

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(BaseTest)
