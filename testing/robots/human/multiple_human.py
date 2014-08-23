#! /usr/bin/env python
"""
This script tests the human model with a pose sensor.
"""

import sys
from morse.testing.testing import MorseTestCase
from pymorse import Morse

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] <your test>.py
try:
    from morse.builder import *
except ImportError:
    pass

class MultipleHumanTest(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        human1 = Human()
        human1.name = "roger"
        human1.append(Pose("pose"))
        human1.disable_keyboard_control()
        human1.use_world_camera()
        human1.translate(x = 5.0)
        human1.add_default_interface('socket')

        human2 = Human()
        human2.name = "rafael"
        human2.append(Pose("pose"))
        human2.disable_keyboard_control()
        human2.use_world_camera()
        human2.translate(x = -5.0)
        human2.add_default_interface('socket')

        human3 = Human()
        human3.name = "novak"
        human3.append(Pose("pose"))
        human3.add_default_interface('socket')

        env = Environment('empty', fastmode = True)

    def test_pose(self):
        with Morse() as morse:
            p1 = morse.human1.pose.get()
            p2 = morse.human2.pose.get()
            p3 = morse.human3.pose.get()

            self.assertAlmostEquals(p1['x'], 5.0, delta=0.01)
            self.assertAlmostEquals(p2['x'], -5.0, delta=0.01)
            self.assertAlmostEquals(p3['x'], 0.0, delta=0.01)
            self.assertAlmostEquals(p1['y'], 0.0, delta=0.01)
            self.assertAlmostEquals(p2['y'], 0.0, delta=0.01)
            self.assertAlmostEquals(p3['y'], 0.0, delta=0.01)

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(MultipleHumanTest)
