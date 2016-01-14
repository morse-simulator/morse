#! /usr/bin/env python
"""
This script tests some of the base functionalities of MORSE.
"""

import sys
import math
import time
from morse.testing.testing import MorseMoveTestCase
from pymorse import Morse

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass


class TimeScale_Test(MorseMoveTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        
        robot = ATRV()

        pose = Pose()
        pose.frequency(60)
        robot.append(pose)
        pose.add_stream('socket')

        motion = MotionVW()
        robot.append(motion)
        motion.add_stream('socket')

        teleport = Teleport()
        robot.append(teleport)
        teleport.add_stream('socket')

        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def send_speed(self, s, morse, v, w, t):
        """
        Send the speed (v, w) to the robot controller during real-time t
        seconds
        Return the simulated time elapsed (in sec)
        """
        start = morse.time()
        self._count = 0
        morse.robot.pose.subscribe(self.count_value)
        s.publish({'v' : v, 'w' : w})
        time.sleep(t)
        s.publish({'v' : 0.0, 'w' : 0.0})
        morse.robot.pose.unsubscribe(self.count_value)
        return morse.time() - start

    def count_value(self, value):
        self._count += 1

    def test_time_scale(self):
        with Morse() as simu:

            res = simu.rpc('time', 'set_time_scale', 1.0)
            if not res:
                logger.warning("time::set_time_scale is not supported on this Blender version")
                return

            simu.deactivate('robot.teleport')

            precision = 0.18
            time_precision = 0.04
        
            # Read the start position, it must be (0.0, 0.0, 0.0)
            self.assertAlmostEqualPositionThenFix(simu, [0.0, 0.0, 0.10, 0.0, 0.0, 0.0], precision)

            v_w = simu.robot.motion

            # Normal speed: one real-time second == one simulated second
            simu_time_elapsed = self.send_speed(v_w, simu, 1.0, 0.0, 2.0)
            # as a consequence, the robot has moved forward by 2 m
            self.assertAlmostEqualPositionThenFix(simu, [2.0, 0.0, 0.10, 0.0, 0.0, 0.0], precision)
            self.assertAlmostEqual(simu_time_elapsed, 2.0, delta=time_precision)
            # We expect to get 60 (default fps) * 2 (s) pose measurement
            self.assertAlmostEqual(self._count, 120, delta = 1)

            simu_time_elapsed = self.send_speed(v_w, simu, -1.0, 0.0, 2.0)
            self.assertAlmostEqualPositionThenFix(simu, [0.0, 0.0, 0.10, 0.0, 0.0, 0.0], precision)
            self.assertAlmostEqual(simu_time_elapsed, 2.0, delta=precision)
            self.assertAlmostEqual(self._count, 120, delta = 1)


            # Slow down the simulation : one real-time second == 500 ms simulated
            simu.rpc('time', 'set_time_scale', 0.5)
            simu_time_elapsed = self.send_speed(v_w, simu, 1.0, 0.0, 2.0)
            # as a consequence, we only have moved forward by 1m in
            # 2 real-second (1 simulated second)
            self.assertAlmostEqualPositionThenFix(simu, [1.0, 0.0, 0.10, 0.0, 0.0, 0.0], precision)
            self.assertAlmostEqual(simu_time_elapsed, 1.0, delta=time_precision)
            # and so we get only 60 measures (as we will simulate 1 sec in simulate time)
            self.assertAlmostEqual(self._count, 60, delta = 1)

            # Accelerate the simulation : one real-time second == 2 simulated second
            simu.rpc('time', 'set_time_scale', 2.0)
            simu_time_elapsed = self.send_speed(v_w, simu, -1.0, 0.0, 2.0)
            # as a consequence, we only have moved downward by 4m in 2 real-second (4 simulated second)
            self.assertAlmostEqualPositionThenFix(simu, [-3.0, 0.0, 0.10, 0.0, 0.0, 0.0], precision)
            self.assertAlmostEqual(simu_time_elapsed, 4.0, delta=precision)
            # alse, we get 240 measures (60 * 4 simulated sec)
            self.assertAlmostEqual(self._count, 240, delta = 2)


########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(TimeScale_Test, time_modes = [TimeStrategies.BestEffort])
