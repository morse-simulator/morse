#! /usr/bin/env python
"""
This script tests the human model with a pose sensor.
"""

import sys
from morse.testing.testing import MorseTestCase
from pymorse import Morse

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|edit] <your test>.py
try:
    from morse.builder import *
except ImportError:
    pass

class MultipleHumanTest(MorseTestCase):
    def setUpEnv(self):
        """ A very simple test scenario, with 3 static human avatars.
        """
        human1 = Human()
        human1.name = 'roger'
        human1.append(Pose("pose"))
        human1.translate(x = 5.0)
        human1.add_default_interface('socket')

        human2 = Human()
        human2.name = 'raphael'
        human2.append(Pose("pose"))
        human2.translate(x = -5.0)
        human2.add_default_interface('socket')

        human3 = Human()
        human3.name = 'novak'
        human3.append(Pose("pose"))
        keyboard = Keyboard()
        keyboard.properties(Speed=1.15)
        human3.append(keyboard)
        human3.add_default_interface('socket')

        env = Environment('empty', fastmode = True)

    def test_pose(self):
        with Morse() as morse:
            p1 = morse.roger.pose.get()
            p2 = morse.raphael.pose.get()
            p3 = morse.novak.pose.get()

            self.assertAlmostEquals(p1['x'], 5.0, delta=0.01)
            self.assertAlmostEquals(p2['x'], -5.0, delta=0.01)
            self.assertAlmostEquals(p3['x'], 0.0, delta=0.01)
            self.assertAlmostEquals(p1['y'], 0.0, delta=0.01)
            self.assertAlmostEquals(p2['y'], 0.0, delta=0.01)
            self.assertAlmostEquals(p3['y'], 0.0, delta=0.01)

    def test_skeletons(self):
        with Morse() as morse:
            s1 = morse.roger.skeleton.joint_states
            s2 = morse.raphael.skeleton.joint_states
            s3 = morse.novak.skeleton.joint_states

            # Check that the pose is the same for all the models (ie, same joints value)
            for j1, j2 in zip(s1.get_state().result().values(), s2.get_state().result().values()):
                self.assertAlmostEquals(j1, j2, delta=0.001)

            for j1, j3 in zip(s1.get_state().result().values(), s3.get_state().result().values()):
                self.assertAlmostEquals(j1, j3, delta=0.001)

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(MultipleHumanTest)
