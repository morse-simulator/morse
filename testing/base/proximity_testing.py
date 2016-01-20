#! /usr/bin/env python
"""
This script tests the proximity sensor in MORSE.
"""

import sys
from morse.testing.testing import MorseTestCase
from pymorse import Morse

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

def send_dest(s, morse, x, y, yaw):
    s.publish({'x' : x, 'y' : y, 'z' : 0, 'yaw' : yaw, 'pitch' : 0.0, 'roll' : 0.0})
    morse.sleep(0.1)

class ProximityTest(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        robot = ATRV()

        proximity = Proximity()
        proximity.translate(z=0.5)
        proximity.properties(Track = "Catch_me")
        proximity.properties(Range = 2.0)
        robot.append(proximity)
        proximity.add_stream('socket')
        proximity.add_service('socket')

        pose = Pose()
        robot.append(pose)
        pose.add_stream('socket')

        motion = Teleport()
        robot.append(motion)
        motion.add_stream('socket')

        target1 = ATRV("Target1")
        target1.properties(Catch_me = True)
        target1.translate(x=10.0, y = 1.0)

        target2 = ATRV("Target2")
        target2.properties(Catch_me2 = True)
        target2.translate(x=10.0, y = -1.0)

        target3 = ATRV("Target3")
        target3.properties(Catch_me = True)
        target3.translate(x=-4.0, y = 0.0)

        env = Environment('empty', fastmode = True)
        env.set_camera_location( (9.952, -14.955, 12.48) )
        env.set_camera_rotation( (0.867, 0, 0.428) )
        env.add_service('socket')

    def test_proximity(self):
        with Morse() as morse:
        
            prox_stream = morse.robot.proximity
            teleport_client = morse.robot.motion

            prox = prox_stream.get()
            self.assertEqual(len(prox['near_objects']), 0)

            # still emtpy
            send_dest(teleport_client, morse, 8.0, 0.0, 0.0)
            prox = prox_stream.get()
            self.assertEqual(len(prox['near_objects']), 0)

            # one more meter, must find target1. target2 is at equal
            # distance but don't have the good tag
            send_dest(teleport_client, morse, 9.0, 0.0, 0.0)
            prox = prox_stream.get()
            self.assertEqual(len(prox['near_objects']), 1)
            self.assertIn('Target1', prox['near_objects'])

            # Don't care about the direction, only check the distance
            send_dest(teleport_client, morse, -2.8, 0.0, 0.0)
            prox = prox_stream.get()
            self.assertEqual(len(prox['near_objects']), 1)
            self.assertIn('Target3', prox['near_objects'])

            # Call the set_range service and check if we can catch the
            # two objects
            prox_stream.set_range(20.0)
            morse.sleep(0.1)
            prox = prox_stream.get()
            self.assertEqual(len(prox['near_objects']), 2)
            self.assertIn('Target1', prox['near_objects'])
            self.assertIn('Target3', prox['near_objects'])

            # Call the set_tracked_tag service and check if we catch
            # target2
            prox_stream.set_tracked_tag('Catch_me2')
            morse.sleep(0.1)
            prox = prox_stream.get()
            self.assertEqual(len(prox['near_objects']), 1)
            self.assertIn('Target2', prox['near_objects'])


########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(ProximityTest)
