import logging
#testrunnerlogger = logging.getLogger("test.runner")
testlogger = logging.getLogger("morsetesting.general")
morselogger = logging.getLogger("morsetesting.morse")

import sys, os
from abc import ABCMeta, abstractmethod
import unittest
import inspect
import tempfile

import subprocess

from morse.testing.exceptions import MorseTestingError
        
class MorseTestRunner(unittest.TextTestRunner):
        
    def setup_logging(self):
        # get the top logger
        logger = logging.getLogger('morsetesting')
        logger.setLevel(logging.DEBUG)

        ch = logging.FileHandler("testing.log")
        ch.setLevel(logging.DEBUG)

        formatter = logging.Formatter('[%(asctime)s %(name)s (%(levelname)s)]   %(message)s')

        ch.setFormatter(formatter)

        logger.addHandler(ch)
        


    def run(self, suite):
        if sys.argv[0].endswith('blender'):
            # If we arrive here from within MORSE, we have probably run
            # morse [exec|run] my_test.py
            # If this case, simply build the environment based on the
            # setUpEnv of the first test.
            
            for test in suite:
                test.setUpEnv()
                break
            
        else:
            self.setup_logging()
            
            return unittest.TextTestRunner.run(self, suite)
                

class MorseTestCase(unittest.TestCase):

    # Make this an abstract class
    __metaclass__ = ABCMeta

    def setUp(self):
        print("Starting test " + self.id())
        self.startmorse(self)

    
    def tearDown(self):
        self.stopmorse()
        
    @abstractmethod
    def setUpEnv(self):
        """ This method must be overloaded by subclasses to define a
        simulation environment.

        The code must follow the :doc:`Builder API <morse/dev/builder>`
        convention (without the import of the `morsebuilder` module which
        is automatically added).
        """
        pass

    def startmorse(self, test_case):
        """ This starts MORSE in a new process, passing the script itself as parameter (to
        build the scene via the Builder API).
        """
        
        temp_builder_script = self.generate_builder_script(test_case)
        try:
            original_script_name = os.path.abspath(inspect.stack()[-1][1])

            try:
                prefix = os.environ['MORSE_ROOT']
            except KeyError:
                prefix=""

            if prefix == "":
                cmd = 'morse'
            else:
                cmd = prefix + "/bin/morse"

            self.morse_process = subprocess.Popen([cmd, 'run', temp_builder_script], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        except OSError as ose:
            print("Error while launching MORSE! Check you can run it from command-line\n" + \
                    " and if you use the $MORSE_ROOT env variable, check it points to a correct " + \
                    " place!")
            raise ose
        
        morse_initialized = False
        for line in iter(self.morse_process.stdout.readline,''):
            morselogger.info(line.rstrip())
            if b"SCENE INITIALIZED" in line:
                morse_initialized = True
                break
        
        if morse_initialized:
            print("MORSE successfully initialized")
        else:
            raise(MorseTestingError("MORSE failed to start!"))
    
    def stopmorse(self):
        """ Cleanly stop MORSE
        """
        import socket
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(("localhost", 4000))
        s.send(b"id1 simulation quit\n")

        for line in iter(self.morse_process.stdout.readline,''):
            morselogger.info(line.rstrip())
            if b"EXITING SIMULATION" in line:
                break

        self.morse_process.terminate()
        print("MORSE stopped")
    
    def generate_builder_script(self, test_case):
        
        tmp_name = ""
        # We need to generate a temp builder file in case of running
        # several test cases with different environment:
        # Blender must be restarted and called again with the right
        # environment.
        with tempfile.NamedTemporaryFile(delete = False) as tmp:
            tmp.write(b"from morse.builder.morsebuilder import *\n")
            tmp.write(b"class MyEnv():\n")
            tmp.write(inspect.getsource(test_case.setUpEnv).encode())
            tmp.write(b"MyEnv().setUpEnv()\n")
            tmp.flush()
            tmp_name = tmp.name
        
        testlogger.info("Created a temporary builder file for test-case " +\
            test_case.__class__.__name__)
        return tmp_name
