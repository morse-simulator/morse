#! /usr/bin/env python
"""
This script tests some of the base functionalities of MORSE.
"""

import sys
from morse.testing.testing import MorseTestCase

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

from pymorse import Morse

class RenamingTest(MorseTestCase):

    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        class MyMorsy(Morsy):
            def __init__(self, name = None):
                Morsy.__init__(self, name)
                self.arm = KukaLWR()
                self.append(self.arm)
                self.pose = Pose()
                self.arm.append(self.pose)
        
        morsy = Morsy()
        arm = KukaLWR()
        arm.name = 'my_kuka'
        morsy.append(arm)
        pose = Pose('my_pose')
        arm.append(pose)

        robot2 = Morsy('mana')
        arm = KukaLWR()
        robot2.append(arm)
        pose = Pose()
        pose.name = 'my_pose2'
        arm.append(pose)

        robot3 = Morsy()
        robot3.name = 'dala'
        arm = KukaLWR('my_kuka')
        robot3.append(arm)
        pose = Pose()
        arm.append(pose)

        # class based
        robot4 = MyMorsy()
        robot5 = MyMorsy('foo')
        robot6 = MyMorsy()
        robot6.name = 'bar'

        # looping
        for i in range(10):
            m = Morsy()
            arm = KukaLWR()
            m.append(arm)
            pose = Pose()
            arm.append(pose)
            AbstractComponent.close_context()

        env = Environment('empty', fastmode = True)
        env.create()

    def test_renaming(self):
        """ Tests the simulator can return the list of robots
        
        This test is guaranteed to be started only when the simulator
        is ready.
        """

        with Morse() as morse:
            p1 = morse.morsy.my_kuka.my_pose
            p2 = morse.mana.arm.my_pose2
            p3 = morse.dala.my_kuka.pose
            p4 = morse.robot4.arm.pose
            p5 = morse.foo.arm.pose
            p6 = morse.bar.arm.pose
            p7 = morse.m.arm.pose
            p8 = morse.m_001.arm.pose
            p9 = morse.m_002.arm.pose
            p10 = morse.m_003.arm.pose
            # ...

            self.assertEqual(len(morse.ms), 10)
            self.assertEqual(morse.m, morse.ms[0])
            self.assertEqual(morse.m_001, morse.ms[1])
            self.assertEqual(morse.m_002, morse.ms[2])



########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(RenamingTest)
