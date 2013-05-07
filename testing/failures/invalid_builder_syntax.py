#! /usr/bin/env python
"""
This script tests how MORSE handles invalid Builder scripts.
"""

import sys
import math
from time import sleep
from morse.testing.testing import MorseBuilderFailureTestCase
from pymorse import Morse

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] <script>.py
try:
    from morse.builder import *
except ImportError:
    pass

class InvalidName_Test(MorseBuilderFailureTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        
        robot = Morsy()
        robot.name = "toto.toto"

        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def test_builder_script(self):
        pass # we only test the parsing of the script


class NoEnvironment_Test(MorseBuilderFailureTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        
        robot = Morsy()

        env = Environment('i_do_not_exist', fastmode = True)
        env.add_service('socket')

    def test_builder_script(self):
        pass # we only test the parsing of the script

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(InvalidName_Test, NoEnvironment_Test)
