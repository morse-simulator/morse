#! /usr/bin/env python
"""
This script tests the Robot's 'friction' property.

http://www.blender.org/documentation/blender_python_api_2_68a_release/bpy.types.MaterialPhysics.html#bpy.types.MaterialPhysics.friction

"""

from morse.testing.testing import MorseTestCase

try:
    # Include this import to be able to use your test file as a regular 
    # builder script, ie, usable with: 'morse [run|exec] <your test>.py
    from morse.builder import *
    from morse.builder.actuators import Light
except ImportError:
    pass

import sys
from pymorse import Morse

def send_speed(s, simu, v=0, w=0, t=0):
    s.publish({'v': v, 'w': w})
    simu.sleep(t)
    s.publish({'v': 0.0, 'w': 0.0})

class FrictionTest(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API. """

        robot1 = ATRV()

        pose = Pose()
        pose.add_stream('socket')
        robot1.append(pose)

        motion = MotionVW()
        motion.add_stream('socket')
        motion.properties(ControlType='Velocity')
        robot1.append(motion)

        robot1.translate(y=-1)
        robot1.set_friction(0)

        robot2 = ATRV()

        pose = Pose()
        pose.add_stream('socket')
        robot2.append(pose)

        motion = MotionVW()
        motion.add_stream('socket')
        motion.properties(ControlType='Velocity')
        robot2.append(motion)

        robot2.translate(y=+1)
        robot2.set_friction(5)

        env = Environment('empty', fastmode = True)

    def test_friction(self):
        with Morse() as simu:

            precision=0.1

            # Read the start position, it must be (0.0, 0.0, 0.0)
            for pose_stream in [simu.robot1.pose, simu.robot2.pose]:
                pose = pose_stream.get()
                self.assertAlmostEqual(pose['x'], 0.0, delta=precision)

            # Read the start position, it must be (0.0, 0.0, 0.0)
            for vw_stream in [simu.robot1.motion, simu.robot2.motion]:
                send_speed(vw_stream, simu, 1.0, 0.0, 2.0)

            pose = simu.robot1.pose.get() # friction = 0 / 100
            self.assertAlmostEqual(pose['x'], 2.0, delta=precision)

            pose = simu.robot2.pose.get() # friction = 5 / 100
            self.assertAlmostEqual(pose['x'], 1.4, delta=precision)

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(FrictionTest)
