#! /usr/bin/env python
from morse.testing.testing import MorseTestCase

try:
    # Include this import to be able to use your test file as a regular 
    # builder script, ie, usable with: 'morse [run|exec] <your test>.py
    from morse.builder import *
except ImportError:
    pass

import os
import sys
import socket
import json
import time
import math
from pymorse import Morse, MorseServerError

def send_dest(s, x, y, yaw):
    s.send(json.dumps({'x' : x, 'y' : y, 'z' : 0, 'yaw' : yaw, 'pitch' : 0.0, 'roll' : 0.0}).encode())
    time.sleep(0.1)

class VictimTest(MorseTestCase):

    def setUpEnv(self):
        robot = Robot('atrv')
        robot.translate(z=0.2)

        victim_detector = Sensor('rosace')
        robot.append(victim_detector)
        victim_detector.configure_mw('socket')
        victim_detector.configure_service('socket')
        victim_detector.properties( Heal_range=1.0, Abilities="1,2,3,4,5")

        victim_detector._blendobj.game.sensors["Radar"].angle = 90.0
        victim_detector._blendobj.game.sensors["Radar"].distance= 2.0

        motion = Actuator('teleport')
        robot.append(motion)
        motion.configure_mw('socket')

        env = Environment('indoors-1/indoor-1')
        env.properties(Temperature='25.0')
        env.configure_service('socket')

        victim = Robot('victim')
        victim.translate(x=10.0, y=0.0)

    def test_victim_interface(self):
        with Morse() as morse:
            victim_stream = morse.stream('Rosace_Sensor')

            port = morse.get_stream_port('Motion_Controller')
            teleport_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            teleport_client.connect(('localhost', port))

            abilities = morse.call_server('Rosace_Sensor', 'get_robot_abilities')
            self.assertEqual(abilities, [1, 2, 3, 4, 5])

            victim_status = victim_stream.get()
            self.assertTrue(len(victim_status['victim_dict']) == 0)

            # Move closer of the victim but not close enough to heal it
            send_dest(teleport_client, 8.0, 0.0, 0.0)

            victim_status = victim_stream.get()
            victim_dict = victim_status['victim_dict']
            self.assertTrue(len(victim_dict) == 1)
            self.assertEqual(victim_dict['Victim']['requirements'], [1, 2, 3])
            self.assertEqual(victim_dict['Victim']['severity'], 10)

            severity = morse.call_server('Rosace_Sensor', 'get_victim_severity')
            self.assertEqual(severity, 10.0)

            requirements = morse.call_server('Rosace_Sensor', 'get_victim_requirements')
            self.assertEqual(requirements, [1, 2, 3])

            with self.assertRaises(MorseServerError):
                morse.call_server('Rosace_Sensor', 'heal')

            # The victim is not detected if not in the field of view of
            # the radar
            send_dest(teleport_client, 8.0, 0.0, math.pi)
            victim_status = victim_stream.get()
            victim_dict = victim_status['victim_dict']
            self.assertTrue(len(victim_dict) == 0)

            with self.assertRaises(MorseServerError):
                morse.call_server('Rosace_Sensor', 'get_victim_severity')

            with self.assertRaises(MorseServerError):
                morse.call_server('Rosace_Sensor', 'get_victim_requirements')

            with self.assertRaises(MorseServerError):
                morse.call_server('Rosace_Sensor', 'heal')

            # Move close enough to be able to heal it
            # XXX why does we need to teleport on y axis
            send_dest(teleport_client, 9.2, -1.2, 0.0)

            victim_status = victim_stream.get()
            victim_dict = victim_status['victim_dict']
            self.assertTrue(len(victim_dict) == 1)
            self.assertEqual(victim_dict['Victim']['requirements'], [1, 2, 3])
            self.assertEqual(victim_dict['Victim']['severity'], 10)

            severity = morse.call_server('Rosace_Sensor', 'get_victim_severity')
            self.assertEqual(severity, 10.0)

            requirements = morse.call_server('Rosace_Sensor', 'get_victim_requirements')
            self.assertEqual(requirements, [1, 2, 3])

            morse.call_server('Rosace_Sensor', 'heal')

            victim_status = victim_stream.get()
            victim_dict = victim_status['victim_dict']
            self.assertTrue(len(victim_dict) == 1)
            self.assertEqual(victim_dict['Victim']['requirements'], [])
            self.assertEqual(victim_dict['Victim']['severity'], 0)

########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(VictimTest)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

