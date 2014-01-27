#! /usr/bin/env python
"""
This script tests the PR2 torso armature joint
"""

from morse.testing.testing import MorseTestCase
import logging; logger = logging.getLogger("morsetesting.general")

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

import sys
import time

from pymorse import Morse

def getjoint(name):
    
    with Morse() as simu:
        joints = simu.pr2.posture.get()

        return joints[name]

class PR2TorsoTest(MorseTestCase):

    def setUpEnv(self):
        from morse.builder.robots import BasePR2
        pr2 = BasePR2()
        pr2.add_stream('socket')

        env = Environment('empty', fastmode=True)
        env.set_camera_rotation([1.0470, 0, 0.7854])

    def test_head_pan(self):
        

        joint = "head_pan"

        self.assertAlmostEquals(getjoint(joint), 0.0, 3)

        with Morse() as simu:

            logger.info("Paning the head 1")
            action = simu.pr2.head.rotate(joint, 1, 1).result() # speed = 1 rad/s
            self.assertAlmostEquals(getjoint(joint), 1, 1)


########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(PR2TorsoTest)
