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

class HumanPoseTest(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """

        human = Human()

        pose = Pose()
        human.append(pose)
        pose.add_stream('socket')

        motion = Waypoint()
        motion.properties(ControlType= 'Position')
        human.append(motion)
        motion.add_stream('socket')
        motion.add_service('socket')

        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def test_pose(self):
        """ Tests we can load the human model, attach a pose sensor, and
        get back the pose.
        """
        with Morse() as morse:

            #Read the start position, it must be (0.0, 0.0, 0.0)
            pose_stream = morse.human.pose
            morse.sleep(1)
            pose = pose_stream.get()
            for key, coord in pose.items():
                if key != 'timestamp':
                    self.assertAlmostEqual(coord, 0.0, delta=0.1)

    def _test_movement(self):
        """ Tests the human can accept an actuator, and that it
        work as expected to move around the human.

        Currently disabled (the waypoint actuator can not move yet the human)
        """
        with Morse() as morse:

            #Read the start position, it must be (0.0, 0.0, 0.0)
            pose_stream = morse.human.pose

            # waypoint controller socket
            v_w_client = morse.human.motion

            v_w_client.publish({'x' : 2.0, 'y': 3.0, 'z': 0.0,
                                'tolerance' : 0.3,
                                'speed' : 1.0})

            morse.sleep(5)
            pose = pose_stream.get()

            self.assertAlmostEqual(pose['x'], 2.0, delta=0.5)
            self.assertAlmostEqual(pose['y'], 3.0, delta=0.5)

    def _test_ik_motion(self):

        IK_TARGET = "ik_target.robot.arm."

        with Morse() as simu:
            self.assertEqual(simu.robot.arm.list_IK_targets(), [IK_TARGET])
            self._check_pose(simu, 0., 0., 1.3105, 0.)

            simu.robot.arm.move_IK_target(IK_TARGET, [0,0,2], None, False).result() # absolute location
            self._check_pose(simu, 0., 0., 1.3105, 0.)

            simu.robot.arm.move_IK_target(IK_TARGET, [1,0,0.3105], None, False).result()
            self._check_pose(simu, 0.778, 0., 0.363, 0.02)

            simu.robot.arm.move_IK_target(IK_TARGET, [1,0,0.3105], [math.pi/2, -math.pi/2, -math.pi], False).result() # arm should be horizontal
            self._check_pose(simu, 1.0, 0., 0.3105, math.radians(90))

            # back to original position
            simu.robot.arm.move_IK_target(IK_TARGET, [0,0,2], [math.pi/2, 0., -math.pi], False).result() # absolute location
            self._check_pose(simu, 0., 0., 1.3105, 0.)

            simu.robot.arm.move_IK_target(IK_TARGET, [-1, 0, -1.6895], None).result() # relative position
            self._check_pose(simu, -0.778, 0., 0.363, -0.02)

            simu.robot.arm.move_IK_target(IK_TARGET, [0.,0.,0.], [0., -math.pi/2, 0.]).result() # relative rotation
            self._check_pose(simu, -1.0, 0., 0.3105, -math.radians(90))


########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(HumanPoseTest)
