#! /usr/bin/env python
"""
We test here the various features of the pymorse bindings:
- automatic creation of robot/components existing in the simulation
- datastream read/write,
- service invokation
"""

from morse.testing.testing import MorseTestCase

try:
    # Include this import to be able to use your test file as a regular 
    # builder script, ie, usable with: 'morse [run|exec] <your test>.py
    from morse.builder import *
except ImportError:
    pass

import sys
import time
from pymorse import Morse

class PymorseTest(MorseTestCase):

    def setUpEnv(self):
        
        ##### Robot1
        # Names come from the variable name

        robot1 = Robot('atrv')
        robot1.translate(0.0,1.0,0.0)

        battery = Sensor('battery')
        battery.configure_mw('socket')
        robot1.append(battery)

        motion = Actuator('waypoint')
        motion.configure_mw('socket')
        motion.configure_service('socket')
        robot1.append(motion)

        ##### Robot2
        # Here, we set explicitely the names via the 'name' property
        robot2 = Robot('atrv')
        robot2.name = "Robi"

        pose = Sensor('pose')
        pose.name = "MyPose"
        pose.configure_mw('socket')
        robot2.append(pose)

        # Environment
        env = Environment('empty', fastmode = True)
        env.configure_service('socket')

    def _test_base(self):
        with Morse() as simu:
            self.assertIn("robots", dir(simu))
            self.assertEquals(set(simu.robots), {'ATRV', 'Robi'})

            self.assertIn('Robi', dir(simu))
            self.assertIn('ATRV', dir(simu))

            self.assertIn('MyPose', simu.Robi)

            self.assertIn('Battery', simu.ATRV)
            self.assertIn('Motion_Controller', simu.ATRV)

            with self.assertRaises(KeyError):
                simu.Robi.ghost_compt

            with self.assertRaises(KeyError):
                simu.Robi.ghost_compt.get()

            self.assertIsNotNone(simu.Robi.MyPose)
            self.assertIsNotNone(simu.ATRV.Battery)
            self.assertIsNotNone(simu.ATRV.Motion_Controller)

    def _test_streams(self):

        with Morse() as morse:

            s = morse.ATRV.Battery

            # Try to read it
            self.assertIsNotNone(s.get())

            s.last()
            s.get()

            motion = morse.ATRV.Motion_Controller

            # Try to write on a stream
            motion.publish({'x' : 10.0, 'y': 5.0, 'z': 0.0, 
                            'tolerance' : 0.5, 
                            'speed' : 1.0})

            time.sleep(1.0)

    def test_services(self):

        with Morse() as morse:

            motion = morse.ATRV.Motion_Controller

            # Should not raise any exception
            motion.goto(1.0, 1.0, 0.0, 0.5, 1.0)

            # Inexistant service
            #TODO: the test blocks here...???
            with self.assertRaises(AttributeError):
                motion.toto()

            # One missing argument
            with self.assertRaises(TypeError):
                motion.goto(5.0, 1.0)

            # Too many arguments
            with self.assertRaises(TypeError):
                motion.goto(10.0, 5.0, 0.0, 0.5, 1.0, 0.0)

            # Wrong type
            # TODO in MORSE: type checking not yet done!
            #with self.assertRaises(ValueError):
            #    motion.goto(10.0, True, 0.0, 0.5, 1.0)

    def test_async_services(self):

        with Morse() as morse:

            motion = morse.ATRV.Motion_Controller


            # Calls to MORSE services return Python's 'futures'.
            req = motion.goto(1.0, 1.0, 0.0, 0.5, 1.0)

            self.assertFalse(req.done())

            res = req.result(10)

            self.assertTrue(req.done())

            self.assertIsNotNone(res) # finished successfully

########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(PymorseTest)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

