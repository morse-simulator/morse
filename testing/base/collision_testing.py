#! /usr/bin/env python
"""
This script tests the Collision sensor.
"""

from morse.testing.testing import MorseTestCase

try:
    # Include this import to be able to use your test file as a regular 
    # builder script, ie, usable with: 'morse [run|exec] <your test>.py
    from morse.builder import *
except ImportError:
    pass

import sys
from pymorse import Morse

def send_speed(s, sim, v=0, w=0, t=0):
    s.publish({'v': v, 'w': w})

    if t:
        sim.sleep(t)
        s.publish({'v': 0.0, 'w': 0.0})

class CollisionTest(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API. """

        dala = ATRV()
        dala.properties(obstacle=1)

        robot = ATRV()

        motion = MotionVW()
        motion.add_stream('socket')
        robot.append(motion)

        collision = Collision()
        collision.properties(only_objects_with_property="obstacle")
        collision.add_stream('socket')
        collision.translate(x = 0.7, z = 0.2)
        robot.append(collision)
        robot.translate(x = -3.0)

        env = Environment('empty', fastmode = True)

    def test_collision(self):
        with Morse() as sim:

            collision = sim.robot.collision.get(timeout=0.1)
            self.assertEqual(collision, None)

            send_speed(sim.robot.motion, sim, 1.0, 0.0, 1.0)


            collision = sim.robot.collision.get(timeout=0.1)
            self.assertEqual(collision, None)
            send_speed(sim.robot.motion, sim, 1.0, 0.0)
            sim.sleep(1.0)
            collision = sim.robot.collision.get(timeout=0.1)
            self.assertNotEqual(collision, None)
            self.assertEqual(collision['objects'], "dala")

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(CollisionTest)
