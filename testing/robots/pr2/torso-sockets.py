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
        joints = simu.pr2.joint_state.get()

        return joints[name]

class PR2TorsoTest(MorseTestCase):

    def setUpEnv(self):
        from morse.builder.robots import BasePR2
        pr2 = BasePR2()
        pr2.add_interface('socket')

        env = Environment('empty', fastmode=True)
        env.aim_camera([1.0470, 0, 0.7854])

    def test_torso_translate(self):
        

        joint = "torso_lift_joint"

        self.assertAlmostEquals(getjoint(joint), 0.0, 3)

        with Morse() as simu:
            
            logger.info("Moving up torso to 0.2m...")
            action = simu.pr2.torso.translate(joint, 0.2, 0.1) # speed = 0.1 m/s

            time.sleep(1)
            logger.info("Should be now at 0.1m...")
            self.assertAlmostEquals(getjoint(joint), 0.1, 1)

            time.sleep(1)
            logger.info("Should be now at 0.2m...")
            self.assertAlmostEquals(getjoint(joint), 0.2, 1)

            # Check we do not move anymore
            logger.info("Should not move anymore...")
            time.sleep(0.5)
            self.assertFalse(action.running())
            self.assertAlmostEquals(getjoint(joint), 0.2, 1)

            # Test limits
            #TODO: currently no way to stop the action when blocking on a joint limit since we can not retreive them from Python (blender 2.64)
            #logger.info("Let's go as high as possible to check my limits...")
            #action = simu.pr2.torso.translate(joint, [0, 1, 0], 0.2) # speed = 0.2 m/s
            #time.sleep(2)
            #self.assertFalse(action.running())
            #self.assertAlmostEquals(getjoint(joint), 0.3, 1)

            # Go back to initial position
            logger.info("Let's go back down...")
            action = simu.pr2.torso.translate(joint, 0, 0.2) # speed = 0.2 m/s
            time.sleep(1.6)
            self.assertFalse(action.running())
            self.assertAlmostEquals(getjoint(joint), 0.0, 1)

            ## Testing set_translation
            logger.info("Moving up torso to 0.2m...")
            simu.pr2.torso.set_translation(joint, 0.2)
            logger.info("Should be now at 0.2m...")
            self.assertAlmostEquals(getjoint(joint), 0.2, 3)

            # Test limits
            logger.info("Let's go as high as possible to check my limits...")
            simu.pr2.torso.set_translation(joint, 1)
            self.assertAlmostEquals(getjoint(joint), 0.31, 3)
            simu.pr2.torso.set_translation(joint, -1)
            self.assertAlmostEquals(getjoint(joint), 0.0, 3)


########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(PR2TorsoTest)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

