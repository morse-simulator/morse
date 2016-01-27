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
import time
import socket
from pymorse import Morse


class SocketSyncTest(MorseTestCase):

    def setUpEnv(self):

        robot = Morsy()

        clock = Clock()
        clock.add_stream('socket')
        robot.append(clock)

        env = Environment('empty')
        env.simulator_frequency(10)
        env.configure_stream_manager('socket', time_sync = True, sync_port = 5000)

    def test_read_clock(self):
        with Morse() as morse:
            clock_stream = morse.robot.clock

            time.sleep(0.5)

            prev_clock = clock_stream.last()
            time.sleep(0.2)
            clock = clock_stream.last()

            self.assertGreater(clock['timestamp'], prev_clock['timestamp'])

            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sync:
                sync.connect(('localhost', 5000))

                # Now the clock is blocked until we triggered it.
                # Checking it :)
                time.sleep(0.2)
                prev_clock = clock_stream.last()
                time.sleep(0.2)
                clock = clock_stream.last()
                self.assertEqual(clock['timestamp'], prev_clock['timestamp'])

                # triggering once
                sync.send(bytes('foo', 'utf-8'))
                clock = clock_stream.get()
                self.assertAlmostEqual(clock['timestamp'] - prev_clock['timestamp'], 0.1, delta = 0.0001)

                # So cool, isn't it :)
                # Close the socket, no more control 

            prev_clock = clock_stream.last()
            time.sleep(0.2)
            clock = clock_stream.last()

            self.assertGreater(clock['timestamp'], prev_clock['timestamp'])

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(SocketSyncTest, time_modes = [TimeStrategies.FixedSimulationStep])
