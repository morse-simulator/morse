#! /usr/bin/env python
"""
This script tests some functionalities of the MOOS middleware connected to
MORSE.
"""
import sys
import math
import subprocess
try:
    from pymoos import pymoos as moos
except:
    import pymoos as moos

from morse.testing.moos import MOOSTestCase
from morse.testing.testing import testlogger
from pymorse import Morse

# Include this import to be able to use your test file as a regular
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

def send_speed_(moos_comms, v, w):
    moos_comms.notify('MORSE_MOTION_VELOCITY', v, -1)
    moos_comms.notify('MORSE_MOTION_YAWRATE', w, -1)

def send_speed(moos_comms, morse, v, w, t):
    send_speed_(moos_comms, v, w)
    morse.sleep(t)
    send_speed_(moos_comms, 0., 0.)

class MOOS_MW_Test(MOOSTestCase):
    """
    MOOS_MW_Test tests simple MOOS middleware communications.

    It sends speed and heading and checks if well applied.
    """

    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """

        robot = ATRV()

        pose = Pose()
        pose.translate(z=-.1)
        robot.append(pose)
        pose.add_stream('moos')

        motion = MotionVW()
        robot.append(motion)
        motion.add_stream('moos')

        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def on_mail(self):
        testlogger.info('on_mail called')
        for msg in self.moos_comms.fetch():
            testlogger.info('iterating thru mail')
            if  (msg.key() == "MORSE_SIM_X") and (msg.is_double()):
                self.x = msg.double() # robot X position [m]
            elif  (msg.key() == "MORSE_SIM_Y") and (msg.is_double()):
                self.y = msg.double() # robot Y position [m]
            elif  (msg.key() == "MORSE_SIM_Z") and (msg.is_double()):
                self.z = msg.double() # robot Z position [m]
            elif  (msg.key() == "MORSE_SIM_ROLL") and (msg.is_double()):
                self.roll = msg.double() # robot roll [rad]
            elif  (msg.key() == "MORSE_SIM_PITCH") and (msg.is_double()):
                self.pitch = msg.double() # robot pitch [rad]
            elif  (msg.key() == "MORSE_SIM_YAW") and (msg.is_double()):
                self.yaw = msg.double() # robot yaw [rad]
        return True

    def on_connect(self):
        testlogger.info('on_connect called')
        return self.moos_comms.register('MORSE_SIM_X',0.) and \
            self.moos_comms.register('MORSE_SIM_Y',0.) and \
            self.moos_comms.register('MORSE_SIM_Z',0.) and \
            self.moos_comms.register('MORSE_SIM_YAW',0.) and \
            self.moos_comms.register('MORSE_SIM_ROLL',0.) and \
            self.moos_comms.register('MORSE_SIM_PITCH',0.)


    def test_vw_controller(self):
        self.x = -1
        self.y=-1.
        self.z=-1.
        self.roll=-1.
        self.pitch=-1.
        self.yaw=-1.

        # launch and set up MOOS comms
        self.moos_comms = moos.comms()
        self.moos_comms.set_on_connect_callback(self.on_connect)
        self.moos_comms.set_on_mail_callback(self.on_mail);
        self.moos_comms.run('127.0.0.1',9000,'iMorse_test_datastream')
        while not self.moos_comms.is_connected():
            pass

        with Morse() as morse:
            # sleep a little to make sure self.moos_comms received the data.
            morse.sleep(1)

            precision = .2
            # read the start position, it must be (0., 0., 0., 0., 0., 0.)
            self.assertAlmostEqual(self.x,      0., delta = precision)
            self.assertAlmostEqual(self.y,      0., delta = precision)
            self.assertAlmostEqual(self.z,      0., delta = precision)
            self.assertAlmostEqual(self.yaw,    0., delta = precision)
            self.assertAlmostEqual(self.roll,   0., delta = precision)
            self.assertAlmostEqual(self.pitch,  0., delta = precision)

            send_speed(self.moos_comms, morse, 1., 0., 2.)
            # read the new position, it must be (2., 0., 0., 0., 0., 0.)
            self.assertAlmostEqual(self.x,      2., delta = precision)
            self.assertAlmostEqual(self.y,      0., delta = precision)
            self.assertAlmostEqual(self.z,      0., delta = precision)
            self.assertAlmostEqual(self.yaw,    0., delta = precision)
            self.assertAlmostEqual(self.roll,   0., delta = precision)
            self.assertAlmostEqual(self.pitch,  0., delta = precision)

            send_speed(self.moos_comms, morse, -1., 0., 2.)
            # read the new position, it must be (0., 0., 0., 0., 0., 0.)
            self.assertAlmostEqual(self.x,      0., delta = precision)
            self.assertAlmostEqual(self.y,      0., delta = precision)
            self.assertAlmostEqual(self.z,      0., delta = precision)
            self.assertAlmostEqual(self.yaw,    0., delta = precision)
            self.assertAlmostEqual(self.roll,   0., delta = precision)
            self.assertAlmostEqual(self.pitch,  0., delta = precision)

            send_speed(self.moos_comms, morse, 1., -math.pi/4., 2.)
            # read the new position, it must be (0., 0., 0., 0., 0., 0.)
            self.assertAlmostEqual(self.x,      4. / math.pi,
                                    delta = precision)
            self.assertAlmostEqual(self.y,      -4. / math.pi,
                                    delta = precision)
            self.assertAlmostEqual(self.z,      0., delta = precision)
            self.assertAlmostEqual(self.yaw,    -math.pi / 2.,
                                    delta = precision)
            self.assertAlmostEqual(self.pitch,  0., delta = precision)
            self.assertAlmostEqual(self.roll,   0., delta = precision)

        # close moos_comms properly
        self.moos_comms.close(True)

if __name__ == "__main__":
    from morse.testing.testing import main
    main(MOOS_MW_Test, time_modes = [TimeStrategies.BestEffort])
