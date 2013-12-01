#! /usr/bin/env python
"""
This script tests the Segway RMP400 robot with differential drive actuator
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

def gradual_speed(s, morse, v, w, t):
    """ Start and finish applying only half of the desired speed """
    tic = t/20.0
    s.publish({'v' : v/4, 'w' : w/4})
    morse.sleep(tic*2)
    s.publish({'v' : v/2, 'w' : w/2})
    morse.sleep(tic*1)
    s.publish({'v' : v, 'w' : w})
    morse.sleep(tic*18)
    s.publish({'v' : v/2, 'w' : w/2})
    morse.sleep(tic*2)
    s.publish({'v' : v/4, 'w' : w/4})
    morse.sleep(tic*2)
    s.publish({'v' : 0.0, 'w' : 0.0})


def send_speed(s, morse, v, w, t):
    #s.publish({'v' : v, 'w' : w})
    #morse.sleep(t)
    #s.publish({'v' : 0.0, 'w' : 0.0})
    gradual_speed(s, morse, v, w, t)
    morse.sleep(1)

def send_service_speed(s, morse, v, w, t):
    s.set_speed(v, w)
    morse.sleep(t)
    s.stop()
    morse.sleep(1)

class Differential_VW_Test(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        
        robot = SegwayRMP400()
        robot.translate(z=0.1)

        pose = Pose()
        robot.append(pose)
        pose.translate(z=-0.1)
        pose.add_stream('socket')
        pose.add_stream('text')

        motion = MotionVWDiff()
        robot.append(motion)
        motion.add_stream('socket')
        motion.add_service('socket')
        
        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def test_vw_controller(self):
        with Morse() as morse:
        
            # Read the start position, it must be (0.0, 0.0, 0.0)
            pose_stream = morse.robot.pose
            pose = pose_stream.get()
            for key,coord in pose.items():
                    if key == 'z':
                        self.assertAlmostEqual(coord, 0.10, delta=0.03)
                    elif key != 'timestamp':
                        self.assertAlmostEqual(coord, 0.0, delta=0.03)

            v_w = morse.robot.motion

            send_speed(v_w, morse, 1.0, 0.0, 2.0)

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 2.0, delta=0.15)
            self.assertAlmostEqual(pose['y'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['z'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['yaw'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=0.15)

            send_speed(v_w, morse, -1.0, 0.0, 2.0)

            pose = pose_stream.get()
            for key,coord in pose.items():
                if key == 'z':
                    self.assertAlmostEqual(coord, 0.10, delta=0.15)
                elif key != 'timestamp':
                    self.assertAlmostEqual(coord, 0.0, delta=0.15)

            """
            send_speed(v_w, morse, 0.0, -math.pi/4.0, 2.0)

            pose = pose_stream.get()
            # for non-null w, we have r = v /  w
            self.assertAlmostEqual(pose['x'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['y'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['z'], 0.10, delta=0.15)
            self.assertAlmostEqual(pose['yaw'], -math.pi/2.0, delta=0.15)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=0.15)

            send_speed(v_w, morse, 0.0, math.pi/4.0, 2.0)

            pose = pose_stream.get()
            for key,coord in pose.items():
                if key == 'z':
                    self.assertAlmostEqual(coord, 0.10, delta=0.15)
                else:
                    self.assertAlmostEqual(coord, 0.0, delta=0.15)
            """


            send_speed(v_w, morse, 1.0, -math.pi/4.0, 2.0)

            pose = pose_stream.get()
            # for non-null w, we have r = v /  w
            self.assertAlmostEqual(pose['x'], 4.0/ math.pi , delta=0.20)
            self.assertAlmostEqual(pose['y'], -4.0/ math.pi , delta=0.20)
            self.assertAlmostEqual(pose['z'], 0.0, delta=0.20)
            self.assertAlmostEqual(pose['yaw'], -math.pi/2.0, delta=0.20)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=0.20)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=0.20)

            send_speed(v_w, morse, 0.5, -math.pi/8.0, 12.0)

            pose = pose_stream.get()
            for key,coord in pose.items():
                if key == 'z':
                    self.assertAlmostEqual(coord, 0.10, delta=0.15)
                elif key != 'timestamp':
                    self.assertAlmostEqual(coord, 0.0, delta=0.20)

            """
            # The robot slips too much to expect a precise position.
            # However, the behaviour seems reasonnable considering its
            # real-life behaviour. 

            send_speed(v_w, morse, -2.0, math.pi/2.0, 3.0)

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 4.0/ math.pi , delta=0.20)
            self.assertAlmostEqual(pose['y'], -4.0/ math.pi , delta=0.20)
            self.assertAlmostEqual(pose['z'], 0.0, delta=0.20)
            self.assertAlmostEqual(pose['yaw'], -math.pi/2.0, delta=0.20)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=0.20)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=0.20)
            """

    def test_vw_service_controller(self):
        with Morse() as morse:
        
            # Read the start position, it must be (0.0, 0.0, 0.0)
            pose_stream = morse.robot.pose
            pose = pose_stream.get()
            for key,coord in pose.items():
                if key == 'z':
                    self.assertAlmostEqual(coord, 0.10, delta=0.02)
                elif key != 'timestamp':
                    self.assertAlmostEqual(coord, 0.0, delta=0.02)

            v_w = morse.robot.motion

            send_service_speed(v_w, morse, 1.0, 0.0, 2.0)

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 2.0, delta=0.15)
            self.assertAlmostEqual(pose['y'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['z'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['yaw'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=0.15)

            send_service_speed(v_w, morse, -1.0, 0.0, 2.0)

            pose = pose_stream.get()
            for key,coord in pose.items():
                if key == 'z':
                    self.assertAlmostEqual(coord, 0.10, delta=0.15)
                elif key != 'timestamp':
                    self.assertAlmostEqual(coord, 0.0, delta=0.15)

            send_service_speed(v_w, morse, 1.0, -math.pi/4.0, 2.0)

            pose = pose_stream.get()
            # for non-null w, we have r = v /  w
            self.assertAlmostEqual(pose['x'], 4.0/ math.pi , delta=0.2)
            self.assertAlmostEqual(pose['y'], -4.0/ math.pi , delta=0.2)
            self.assertAlmostEqual(pose['z'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['yaw'], -math.pi/2.0, delta=0.15)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=0.15)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=0.15)

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(Differential_VW_Test)
