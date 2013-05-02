"""
We test here the various features of the pymorse bindings:
- automatic creation of robot/components existing in the simulation
- datastream read/write,
- service invokation
"""

from morse.testing.testing import MorseTestCase
import logging;logger = logging.getLogger("morsetesting.general")

try:
    # Include this import to be able to use your test file as a regular 
    # builder script, ie, usable with: 'morse [run|exec] <your test>.py
    from morse.builder import *
except ImportError:
    pass

import sys
import time
from pymorse import Morse, MorseServicePreempted
from concurrent.futures._base import TimeoutError
class PymorseTest(MorseTestCase):

    def setUpEnv(self):
        
        ##### Robot1
        # Names come from the variable name

        robot1 = ATRV()
        robot1.translate(0.0,1.0,0.0)

        battery = Battery()
        battery.add_stream('socket')
        robot1.append(battery)

        motion = Waypoint()
        motion.add_stream('socket')
        motion.configure_service('socket')
        robot1.append(motion)

        ##### Robot2
        # Here, we set explicitely the names via the 'name' property
        robot2 = ATRV("Robi")

        pose = Pose("MyPose")
        pose.add_stream('socket')
        robot2.append(pose)

        # Environment
        env = Environment('empty', fastmode = True)
        env.configure_service('socket')

    def _test_base(self):
        with Morse() as simu:
            self.assertIn("robots", dir(simu))
            self.assertEquals(set(simu.robots), {'robot1', 'Robi'})

            self.assertIn('Robi', dir(simu))
            self.assertIn('robot1', dir(simu))

            self.assertIn('MyPose', simu.Robi)

            self.assertIn('battery', simu.robot1)
            self.assertIn('motion', simu.robot1)

            with self.assertRaises(KeyError):
                simu.Robi.ghost_compt

            with self.assertRaises(KeyError):
                simu.Robi.ghost_compt.get()

            self.assertIsNotNone(simu.Robi.MyPose)
            self.assertIsNotNone(simu.robot1.battery)
            self.assertIsNotNone(simu.robot1.motion)

    def _test_streams(self):

        with Morse() as morse:

            s = morse.robot1.battery

            # Try to read it
            self.assertIsNotNone(s.get())

            s.last()
            s.get()

            motion = morse.robot1.motion

            # Try to write on a stream
            motion.publish({'x' : 10.0, 'y': 5.0, 'z': 0.0, 
                            'tolerance' : 0.5, 
                            'speed' : 1.0})

            time.sleep(1.0)

    def test_services(self):

        with Morse() as morse:

            motion = morse.robot1.motion

            # Should not raise any exception
            req = motion.goto(1.0, 1.0, 0.0, 0.5, 1.0)

            self.assertFalse(req.done())

            res = req.result(2)

            self.assertTrue(req.done())

            self.assertIsNotNone(res) # finished successfully

            # Inexistant service
            with self.assertRaises(AttributeError):
                motion.toto()

            # One missing argument
            fut = motion.goto(5.0, 1.0)
            self.assertEqual(type(fut.exception(1)), TypeError)

            # Too many arguments
            fut = motion.goto(10.0, 5.0, 0.0, 0.5, 1.0, 0.0)
            self.assertEqual(type(fut.exception(1)), TypeError)

            # Wrong type
            # TODO in MORSE: type checking not yet done!
            #with self.assertRaises(ValueError):
            #    motion.goto(10.0, True, 0.0, 0.5, 1.0)


            ## Testing service cancellation
            logger.info("Starting new movement")
            act = motion.goto(1.0, 2.0, 0.0, 0.1, 0.1)
            self.assertTrue(act.running())
            self.assertFalse(act.done())
            logger.info("Ok")

            logger.info("Cancelling it...")
            act.cancel()
            self.assertEqual(type(act.exception(2)), MorseServicePreempted) # action cancelled
            self.assertFalse(act.running())
            self.assertTrue(act.done())
            logger.info("Ok")


            logger.info("Testing double cancellation")
            act.cancel() # should not trigger anything
            logger.info("Ok")

            ## Testing preemption
            logger.info("Starting new movement")
            act1 = motion.goto(2.0, 1.0, 0.0, 0.1, 0.1)
            self.assertTrue(act1.running())
            self.assertFalse(act1.done())
            logger.info("Ok")

            logger.info("Preempting movement with another one")
            act2 = motion.goto(0.0, 1.0, 0.0, 0.1, 0.1)
            self.assertEqual(type(act1.exception(2)), MorseServicePreempted) # action preempted
            self.assertFalse(act1.running())
            self.assertTrue(act1.done())
            self.assertTrue(act2.running())
            logger.info("Ok")

            act2.cancel()

########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(PymorseTest)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

