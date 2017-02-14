""" MOOS-based MorseTestCase (unit-tests)

The main feature is to launch MOOSDB at start.
"""

from morse.testing.testing import MorseTestCase, testlogger

import os
import sys
import subprocess

try:
    import pymoos
except ImportError as error:
    testlogger.error("Could not find pymoos. Is it installed?")
    raise error

class MOOSTestCase(MorseTestCase):
    def setUpMw(self):
        try:
            self.moosdb_process = subprocess.Popen(['MOOSDB'])
        except OSError as ose:
            testlogger.error("Error while launching MOOSDB ! Is core-moos installed?\n")
            raise ose

    def tearDownMw(self):
        self.moosdb_process.kill()
