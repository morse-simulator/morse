#! /usr/bin/env python
from morse.testing.testing import MorseTestCase

try:
    # Include this import to be able to use your test file as a regular 
    # builder script, ie, usable with: 'morse [run|exec] <your test>.py
    from morse.builder import *
except ImportError:
    pass

import sys
from pymorse import Morse

def send_dest(s, morse, x, y, yaw):
    s.publish({'x' : x, 'y' : y, 'z' : 0, 'yaw' : yaw, 'pitch' : 0.0, 'roll' : 0.0})
    morse.sleep(0.1)

class ThermometerTest(MorseTestCase):

    def setUpEnv(self):
        robot = ATRV()
        robot.translate(z=0.2)

        thermometer = Thermometer()
        robot.append(thermometer)
        thermometer.properties(DefaultTemperature='25.0')
        thermometer.add_interface('socket')

        motion = Teleport()
        robot.append(motion)
        motion.add_interface('socket')

        env = Environment('land-1/rosace_1', fastmode = True)
        env.add_service('socket')

    def test_temperature(self):
        with Morse() as morse:
            temp_stream = morse.robot.thermometer
            teleport_client = morse.robot.motion

            o = temp_stream.get()
            self.assertAlmostEqual(o['temperature'], 25.0, delta=0.01)


            send_dest(teleport_client, morse, 40.0, 0.0, 0.0)

            # We are nearer of the fire so the temperature is expected
            # to be hotter.
            o = temp_stream.get()
            temp = o['temperature']
            self.assertGreater(temp, 28.0)

            # It must be hotter and hotter ... 
            morse.sleep(5.0)
            o = temp_stream.get()
            self.assertGreater(o['temperature'], temp)
            temp = o['temperature']

            morse.sleep(5.0)
            o = temp_stream.get()
            self.assertGreater(o['temperature'], temp)
            temp = o['temperature']





########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(ThermometerTest)
