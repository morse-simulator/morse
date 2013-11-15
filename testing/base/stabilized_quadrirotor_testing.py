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

def send_ctrl(s, theta, phi, psi, h):
    s.publish({'theta_c' : theta, 'phi_c' : phi , 'psi_c' : psi, 'h_c' : h})


class StabilizedQuadrirotorTest(MorseTestCase):

    def setUpEnv(self):
        
        robot = QUAD2012('robot')
        
        pose = Pose()
        pose.add_stream('socket')
        robot.append(pose)

        motion = StabilizedQuadrotor('motion')
        robot.append(motion)
        motion.add_stream('socket')

        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def test_theta_c_control(self):
        with Morse() as morse:
            pose_stream = morse.robot.pose
            cmd_client = morse.robot.motion

            pose = pose_stream.get()

            send_ctrl(cmd_client, 0.0, 0.0, 0.0, 10.0)
            morse.sleep(3.0)

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 0.0, delta=0.2)
            self.assertAlmostEqual(pose['y'], 0.0, delta=0.2)
            self.assertAlmostEqual(pose['z'], 10.0, delta = 0.2)

            # theta_c permits to control the acceleration on x
            send_ctrl(cmd_client, 0.1, 0.0, 0.0, 10.0)
            morse.sleep(1.0)
            pose1 = pose_stream.get()
            dx1 = 0.4
            self.assertAlmostEqual(pose1['x'], dx1, delta=0.2)
            self.assertAlmostEqual(pose1['y'], 0.0, delta=0.2)
            self.assertAlmostEqual(pose1['z'], 10.0, delta=0.2)
            self.assertAlmostEqual(pose1['yaw'], 0.0, delta=0.02)
            self.assertAlmostEqual(pose1['pitch'], 0.1, delta=0.1)
            self.assertAlmostEqual(pose1['roll'], 0.0, delta=0.0)
            
            # acceleration is constant as long ass we waintain theta_c
            morse.sleep(1.0)
            pose2 = pose_stream.get()
            dx2 = 1.4
            self.assertAlmostEqual(pose2['x'], dx1 + dx2, delta=0.2)
            self.assertAlmostEqual(pose2['y'], 0.0, delta=0.2)
            self.assertAlmostEqual(pose2['z'], 10.0, delta=0.2)
            self.assertAlmostEqual(pose2['yaw'], 0.0, delta=0.02)
            self.assertAlmostEqual(pose2['pitch'], 0.1, delta=0.1)
            self.assertAlmostEqual(pose2['roll'], 0.0, delta=0.0)

            # if we setup theta_c to 0, acceleration on x is now null,
            # so v_x is constant (no friction)
            send_ctrl(cmd_client, 0.0, 0.0, 0.0, 10.0)
            morse.sleep(1.0)
            pose3 = pose_stream.get()
            dx3 = 2.2
            self.assertAlmostEqual(pose3['x'], dx1 + dx2 + dx3 , delta=0.2)
            self.assertAlmostEqual(pose3['y'], 0.0, delta=0.2)
            self.assertAlmostEqual(pose3['z'], 10.0, delta=0.2)
            self.assertAlmostEqual(pose3['yaw'], 0.0, delta=0.02)
            self.assertAlmostEqual(pose3['pitch'], 0.0, delta=0.1)
            self.assertAlmostEqual(pose3['roll'], 0.0, delta=0.0)

            morse.sleep(1.0)
            pose4 = pose_stream.get()
            self.assertAlmostEqual(pose4['x'], dx1 + dx2 + 2 * dx3, delta=0.2)
            self.assertAlmostEqual(pose4['y'], 0.0, delta=0.2)
            self.assertAlmostEqual(pose4['z'], 10.0, delta=0.2)
            self.assertAlmostEqual(pose4['yaw'], 0.0, delta=0.02)
            self.assertAlmostEqual(pose4['pitch'], 0.0, delta=0.1)
            self.assertAlmostEqual(pose4['roll'], 0.0, delta=0.0)

            # adding a negative value to theta_c gives a negative
            # acceleration on x, leading to some stabilization at some
            # point t
            send_ctrl(cmd_client, -0.1, 0.0, 0.0, 10.0)
            last_x = 0.0
            pose = pose_stream.get()
            cur_x = pose['x']
            while cur_x - last_x  > 0.005:
                morse.sleep(0.01)
                last_x = cur_x
                pose = pose_stream.get()
                cur_x = pose['x']
            send_ctrl(cmd_client, 0.0, 0.0, 0.0, 10.0)

            morse.sleep(0.2)
            pose = pose_stream.get()
            morse.sleep(1.0)
            pose1 = pose_stream.get()
            self.assertAlmostEqual(pose['x'] - pose1['x'], 0.0, delta=0.2)
            self.assertAlmostEqual(pose['y'] - pose1['y'], 0.0, delta=0.05)
            self.assertAlmostEqual(pose['z'] - pose1['z'], 0.0, delta=0.05)
            self.assertAlmostEqual(pose['yaw'] - pose1['yaw'], 0.0, delta=0.05)
            self.assertAlmostEqual(pose['pitch'] - pose1['pitch'], 0.0, delta=0.05)
            self.assertAlmostEqual(pose['roll'] - pose1['roll'], 0.0, delta=0.05)

    def test_phi_c_control(self):
        with Morse() as morse:
            pose_stream = morse.robot.pose
            cmd_client = morse.robot.motion

            pose = pose_stream.get()

            send_ctrl(cmd_client, 0.0, 0.0, 0.0, 10.0)
            morse.sleep(3.0)

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 0.0, delta=0.2)
            self.assertAlmostEqual(pose['y'], 0.0, delta=0.2)
            self.assertAlmostEqual(pose['z'], 10.0, delta = 0.2)

            # phi_c permits to control the acceleration on y
            send_ctrl(cmd_client, 0.0, 0.1, 0.0, 10.0)
            morse.sleep(1.0)
            pose1 = pose_stream.get()
            dy1 = -0.4
            self.assertAlmostEqual(pose1['y'], dy1, delta=0.2)
            self.assertAlmostEqual(pose1['x'], 0.0, delta=0.2)
            self.assertAlmostEqual(pose1['z'], 10.0, delta=0.2)
            self.assertAlmostEqual(pose1['yaw'], 0.0, delta=0.02)
            self.assertAlmostEqual(pose1['roll'], 0.1, delta=0.1)
            self.assertAlmostEqual(pose1['pitch'], 0.0, delta=0.0)
            
            # acceleration is constant as long ass we waintain theta_c
            morse.sleep(1.0)
            pose2 = pose_stream.get()
            dy2 = -1.4
            self.assertAlmostEqual(pose2['y'], dy1 + dy2, delta=0.2)
            self.assertAlmostEqual(pose2['x'], 0.0, delta=0.2)
            self.assertAlmostEqual(pose2['z'], 10.0, delta=0.2)
            self.assertAlmostEqual(pose2['yaw'], 0.0, delta=0.02)
            self.assertAlmostEqual(pose2['roll'], 0.1, delta=0.1)
            self.assertAlmostEqual(pose2['pitch'], 0.0, delta=0.0)

            # if we setup phi_c to 0, acceleration on y is now null,
            # so v_y is constant (no friction)
            send_ctrl(cmd_client, 0.0, 0.0, 0.0, 10.0)
            morse.sleep(1.0)
            pose3 = pose_stream.get()
            dy3 = -2.2
            self.assertAlmostEqual(pose3['y'], dy1 + dy2 + dy3 , delta=0.2)
            self.assertAlmostEqual(pose3['x'], 0.0, delta=0.2)
            self.assertAlmostEqual(pose3['z'], 10.0, delta=0.2)
            self.assertAlmostEqual(pose3['yaw'], 0.0, delta=0.02)
            self.assertAlmostEqual(pose3['pitch'], 0.0, delta=0.0)
            self.assertAlmostEqual(pose3['roll'], 0.0, delta=0.1)

            morse.sleep(1.0)
            pose4 = pose_stream.get()
            self.assertAlmostEqual(pose4['y'], dy1 + dy2 + 2 * dy3, delta=0.2)
            self.assertAlmostEqual(pose4['x'], 0.0, delta=0.2)
            self.assertAlmostEqual(pose4['z'], 10.0, delta=0.2)
            self.assertAlmostEqual(pose4['yaw'], 0.0, delta=0.0)
            self.assertAlmostEqual(pose4['pitch'], 0.0, delta=0.0)
            self.assertAlmostEqual(pose4['roll'], 0.0, delta=0.1)

            # adding a negative value to theta_c gives a negative
            # acceleration on x, leading to some stabilization at some
            # point t
            send_ctrl(cmd_client, 0.0, -0.1, 0.0, 10.0)
            last_y = 0.0
            pose = pose_stream.get()
            cur_y = pose['y']
            while cur_y - last_y  < -0.005:
                morse.sleep(0.01)
                last_y = cur_y
                pose = pose_stream.get()
                cur_y = pose['y']
            send_ctrl(cmd_client, 0.0, 0.0, 0.0, 10.0)

            morse.sleep(0.2)
            pose = pose_stream.get()
            morse.sleep(1.0)
            pose1 = pose_stream.get()
            self.assertAlmostEqual(pose['x'] - pose1['x'], 0.0, delta=0.05)
            self.assertAlmostEqual(pose['y'] - pose1['y'], 0.0, delta=0.2)
            self.assertAlmostEqual(pose['z'] - pose1['z'], 0.0, delta=0.05)
            self.assertAlmostEqual(pose['yaw'] - pose1['yaw'], 0.0, delta=0.05)
            self.assertAlmostEqual(pose['pitch'] - pose1['pitch'], 0.0, delta=0.05)
            self.assertAlmostEqual(pose['roll'] - pose1['roll'], 0.0, delta=0.05)


    def test_psi_c_control(self):
        with Morse() as morse:
            pose_stream = morse.robot.pose
            cmd_client = morse.robot.motion

            pose = pose_stream.get()

            z = 12.0
            send_ctrl(cmd_client, 0.0, 0.0, 0.0, z)
            morse.sleep(3.0)
            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 0.0, delta=0.1)
            self.assertAlmostEqual(pose['y'], 0.0, delta=0.1)
            self.assertAlmostEqual(pose['yaw'], 0.0, delta=0.1)
            self.assertAlmostEqual(pose['z'], z, delta = 0.2)

            # psi correspond to a delta yaw cmd
            send_ctrl(cmd_client, 0.0, 0.0, 0.1, z)
            morse.sleep(1.0)
            pose = pose_stream.get()
            delta_yaw = -0.69
            self.assertAlmostEqual(pose['x'], 0.0, delta=0.1)
            self.assertAlmostEqual(pose['y'], 0.0, delta=0.1)
            self.assertAlmostEqual(pose['z'], z, delta=0.2)
            self.assertAlmostEqual(pose['yaw'], delta_yaw, delta=0.1)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=0.1)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=0.1)

            morse.sleep(1.0)
            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 0.0, delta=0.1)
            self.assertAlmostEqual(pose['y'], 0.0, delta=0.1)
            self.assertAlmostEqual(pose['z'], z, delta=0.2)
            self.assertAlmostEqual(pose['yaw'], 2 * delta_yaw, delta=0.1)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=0.1)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=0.1)

            send_ctrl(cmd_client, 0.0, 0.0, 0.0, z)
            morse.sleep(1.0)
            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 0.0, delta=0.1)
            self.assertAlmostEqual(pose['y'], 0.0, delta=0.1)
            self.assertAlmostEqual(pose['z'], z, delta=0.2)
            self.assertAlmostEqual(pose['yaw'], 2 * delta_yaw, delta=0.2)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=0.1)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=0.1)

            send_ctrl(cmd_client, 0.0, 0.0, -0.1, z)
            morse.sleep(1.0)
            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 0.0, delta=0.1)
            self.assertAlmostEqual(pose['y'], 0.0, delta=0.1)
            self.assertAlmostEqual(pose['z'], z, delta=0.2)
            self.assertAlmostEqual(pose['yaw'], delta_yaw, delta=0.2)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=0.1)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=0.1)



########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(StabilizedQuadrirotorTest)
