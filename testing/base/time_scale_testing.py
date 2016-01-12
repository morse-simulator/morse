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
        start = time.time()
        self._count = 0
        morse.robot.pose.subscribe(self.count_value)
        s.publish({'v' : v, 'w' : w})
        morse.sleep(t)
        s.publish({'v' : 0.0, 'w' : 0.0})
        morse.robot.pose.unsubscribe(self.count_value)
        return time.time() - start

    def count_value(self, value):
        self._count += 1

    def test_time_scale(self):
        with Morse() as simu:

            res = simu.rpc('time', 'set_time_scale', 1.0)
            if not res:
                logger.warning("time::set_time_scale is not supported on this Blender version")
                return

            simu.deactivate('robot.teleport')

            precision = 0.125
            time_precision = 0.02
        
            # Read the start position, it must be (0.0, 0.0, 0.0)
            self.assertAlmostEqualPositionThenFix(simu, [0.0, 0.0, 0.10, 0.0, 0.0, 0.0], precision)

            v_w = simu.robot.motion

            real_time_elapsed = self.send_speed(v_w, simu, 1.0, 0.0, 2.0)
            self.assertAlmostEqualPositionThenFix(simu, [2.0, 0.0, 0.10, 0.0, 0.0, 0.0], precision)
            self.assertAlmostEqual(real_time_elapsed, 2.0, delta=time_precision)
            self.assertEqual(self._count, 121)

            real_time_elapsed = self.send_speed(v_w, simu, -1.0, 0.0, 2.0)
            self.assertAlmostEqualPositionThenFix(simu, [0.0, 0.0, 0.10, 0.0, 0.0, 0.0], precision)
            self.assertAlmostEqual(real_time_elapsed, 2.0, delta=precision)
            self.assertEqual(self._count, 121)


            simu.rpc('time', 'set_time_scale', 0.5)
            real_time_elapsed = self.send_speed(v_w, simu, 1.0, 0.0, 2.0)
            self.assertAlmostEqualPositionThenFix(simu, [2.0, 0.0, 0.10, 0.0, 0.0, 0.0], precision)
            self.assertAlmostEqual(real_time_elapsed, 4.0, delta=time_precision)
            self.assertEqual(self._count, 241)

            simu.rpc('time', 'set_time_scale', 2.0)
            real_time_elapsed = self.send_speed(v_w, simu, -1.0, 0.0, 2.0)
            self.assertAlmostEqualPositionThenFix(simu, [0.0, 0.0, 0.10, 0.0, 0.0, 0.0], precision)
            self.assertAlmostEqual(real_time_elapsed, 1.0, delta=precision)
            self.assertEqual(self._count, 61)


########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(TimeScale_Test, time_modes = [TimeStrategies.BestEffort])
