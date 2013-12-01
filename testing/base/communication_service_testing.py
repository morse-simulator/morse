#! /usr/bin/env python

import sys
import math
from morse.testing.testing import MorseTestCase
from pymorse import Morse, MorseServiceFailed

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass


class Communication_Service_Testing(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        mana = ATRV()

        minnie = ATRV()
        minnie.translate(x = 10.0)

        munu = ATRV()
        munu.translate(x = -10.0)

        env = Environment('indoors-1/boxes', fastmode=True)
        env.add_service('socket')


    def test_semantic_camera(self):
        """ This test is guaranteed to be started only when the simulator
        is ready.
        """
        with Morse() as morse:
            # not enough argument
            with self.assertRaises(MorseServiceFailed):
                res = morse.rpc('communication', 'distance_and_view')

            # unknown robot does not exit
            with self.assertRaises(MorseServiceFailed):
                res = morse.rpc('communication', 'distance_and_view', 'mana', 'unknow_robot')
                
            res = morse.rpc('communication', 'distance_and_view', 'mana', 'minnie')
            self.assertAlmostEquals(res[0], 10.0, delta=0.01)
            self.assertTrue(res[1])

            res = morse.rpc('communication', 'distance_and_view', 'mana', 'munu')
            self.assertAlmostEquals(res[0], 10.0, delta=0.01)
            self.assertFalse(res[1])

            res = morse.rpc('communication', 'distance_and_view', 'minnie', 'munu')
            self.assertAlmostEquals(res[0], 20.0, delta=0.01)
            self.assertFalse(res[1])

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(Communication_Service_Testing)
