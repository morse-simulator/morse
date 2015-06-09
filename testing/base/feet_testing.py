#! /usr/bin/env python
"""
This script tests the 'data stream' oriented feature of the socket interface.
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


class FeetModifierTest(MorseTestCase):

    def setUpEnv(self):
        
        robot = ATRV()
        robot.translate(10.0, 8.0, 0.0)
        
        pose = Pose()
        pose.add_stream('socket')
        robot.append(pose)

        pose_mod = Pose()
        pose_mod.add_stream('socket')
        pose_mod.alter('feet')
        robot.append(pose_mod)

        teleport = Teleport()
        teleport.add_stream('socket')
        robot.append(teleport)

        teleport_mod = Teleport()
        teleport_mod.add_stream('socket')
        robot.append(teleport_mod)
        teleport_mod.alter('feet')

        env = Environment('empty', fastmode = True)
        env.add_service('socket')


    def test_feet_modifier(self):
        """ Test if we can connect to the pose data stream, and read from it.
        """

        with Morse() as morse:
            pose_stream = morse.robot.pose
            pose_mod_stream = morse.robot.pose_mod
            teleport_stream = morse.robot.teleport
            teleport_mod_stream = morse.robot.teleport_mod

            pos = pose_stream.get()
            pos_mod = pose_mod_stream.get()

            precision = 0.02
            self.assertAlmostEqual(pos['x'], 10.0, delta=precision)
            self.assertAlmostEqual(pos['y'], 8.0, delta=precision)
            # Z = 0.1 : pose of the ATRV's center relative to the world
            self.assertAlmostEqual(pos['z'], 0.1, delta=precision)

            self.assertAlmostEqual(pos_mod['x'], 32.8084, delta=precision)
            self.assertAlmostEqual(pos_mod['y'], 26.246, delta=precision)
            self.assertAlmostEqual(pos_mod['z'], .3280, delta=precision)

            teleport_stream.publish({'x' : 100.0, 'y' : 200.0, 'z' : 50.0, 'yaw' : 0.0, 'pitch' : 0.0, 'roll' : 0.0})
            morse.sleep(0.01)

            pos = pose_stream.get()
            pos_mod = pose_mod_stream.last()

            self.assertAlmostEqual(pos['x'], 100.0, delta=precision)
            self.assertAlmostEqual(pos['y'], 200.0, delta=precision)
            self.assertAlmostEqual(pos['z'], 50.0, delta=precision)
            self.assertAlmostEqual(pos_mod['x'], 328.084, delta=precision)
            self.assertAlmostEqual(pos_mod['y'], 656.168, delta=precision)
            self.assertAlmostEqual(pos_mod['z'], 164.042, delta=precision)

            morse.deactivate('robot.teleport')
            teleport_mod_stream.publish({'x': 32.8084, 'y': 26.246, 'z': 0.3280,  'yaw' : 0.0, 'pitch' : 0.0, 'roll': 0.0})
            morse.sleep(0.03)

            pos = pose_stream.get()
            pos_mod = pose_mod_stream.last()

            self.assertAlmostEqual(pos['x'], 10.0, delta=0.15)
            self.assertAlmostEqual(pos['y'], 8.0, delta=0.15)
            self.assertAlmostEqual(pos['z'], 0.1, delta=0.15)
            self.assertAlmostEqual(pos_mod['x'], 32.8084, delta=precision)
            self.assertAlmostEqual(pos_mod['y'], 26.246, delta=precision)
            self.assertAlmostEqual(pos_mod['z'], .3280, delta=precision)

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(FeetModifierTest)
