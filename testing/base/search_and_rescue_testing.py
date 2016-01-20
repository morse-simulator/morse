#! /usr/bin/env python
from morse.testing.testing import MorseTestCase

try:
    # Include this import to be able to use your test file as a regular 
    # builder script, ie, usable with: 'morse [run|exec] <your test>.py
    from morse.builder import *
except ImportError:
    pass

import sys
import math
from pymorse import Morse, MorseServiceFailed

def send_dest(s, morse, x, y, yaw):
    s.publish({'x' : x, 'y' : y, 'z' : 0, \
                       'yaw' : yaw, 'pitch' : 0.0, 'roll' : 0.0})
    morse.sleep(0.1)

class SearchAndRescueTest(MorseTestCase):

    def setUpEnv(self):
        robot = ATRV()
        robot.translate(z=0.2)

        victim_detector = SearchAndRescue()
        robot.append(victim_detector)
        victim_detector.add_stream('socket')
        victim_detector.add_service('socket')
        victim_detector.properties( Heal_range=1.0, Abilities="1,2,3,4,5",
                                    Angle = 90.0, Distance = 2.0)

        motion = Teleport()
        robot.append(motion)
        motion.add_stream('socket')

        env = Environment('empty', fastmode = True)
        env.properties(Temperature='25.0')
        env.add_service('socket')

        victim = Victim()
        victim.translate(x=10.0, y=0.0)

    def test_victim_interface(self):
        with Morse() as morse:
            victim_stream = morse.robot.victim_detector
            teleport_client = morse.robot.motion

            abilities = morse.rpc('robot.victim_detector', 'get_robot_abilities')
            self.assertEqual(abilities, [1, 2, 3, 4, 5])

            victim_status = victim_stream.get()
            self.assertEqual(len(victim_status['victim_dict']), 0)

            # Move closer of the victim but not close enough to heal it
            send_dest(teleport_client, morse, 8.0, 0.0, 0.0)

            victim_status = victim_stream.get()
            victim_dict = victim_status['victim_dict']
            self.assertEqual(len(victim_dict), 1)
            self.assertEqual(victim_dict['Victim']['requirements'], [1, 2, 3])
            self.assertEqual(victim_dict['Victim']['severity'], 10)

            severity = morse.rpc('robot.victim_detector', 'get_victim_severity')
            self.assertEqual(severity, 10.0)

            requirements = morse.rpc('robot.victim_detector', 'get_victim_requirements')
            self.assertEqual(requirements, [1, 2, 3])

            with self.assertRaises(MorseServiceFailed):
                morse.rpc('robot.victim_detector', 'heal')

            # The victim is not detected if not in the field of view of
            # the radar
            send_dest(teleport_client, morse, 8.0, 0.0, math.pi)
            victim_status = victim_stream.get()
            victim_dict = victim_status['victim_dict']
            self.assertEqual(len(victim_dict), 0)

            with self.assertRaises(MorseServiceFailed):
                morse.rpc('robot.victim_detector', 'get_victim_severity')

            with self.assertRaises(MorseServiceFailed):
                morse.rpc('robot.victim_detector', 'get_victim_requirements')

            with self.assertRaises(MorseServiceFailed):
                morse.rpc('robot.victim_detector', 'heal')

            # Move close enough to be able to heal it
            send_dest(teleport_client, morse, 9.2, 0.0, 0.0)

            victim_status = victim_stream.get()
            victim_dict = victim_status['victim_dict']
            self.assertEqual(len(victim_dict), 1)
            self.assertEqual(victim_dict['Victim']['requirements'], [1, 2, 3])
            self.assertEqual(victim_dict['Victim']['severity'], 10)

            severity = morse.rpc('robot.victim_detector', 'get_victim_severity')
            self.assertEqual(severity, 10.0)

            requirements = morse.rpc('robot.victim_detector', 'get_victim_requirements')
            self.assertEqual(requirements, [1, 2, 3])

            morse.rpc('robot.victim_detector', 'heal')

            morse.sleep(0.2)
            victim_status = victim_stream.get()
            victim_dict = victim_status['victim_dict']
            self.assertEqual(len(victim_dict), 1)
            self.assertEqual(victim_dict['Victim']['requirements'], [])
            self.assertEqual(victim_dict['Victim']['severity'], 0)

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(SearchAndRescueTest)
