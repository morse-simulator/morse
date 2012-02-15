import logging
#testrunnerlogger = logging.getLogger("test.runner")
testlogger = logging.getLogger("morsetesting.general")

import sys, os
from abc import ABCMeta, abstractmethod
import unittest
import inspect
import tempfile
from time import sleep
import threading # Used to be able to timeout when waiting for Blender initialization
import subprocess

from morse.testing.exceptions import MorseTestingError

BLENDER_INITIALIZATION_TIMEOUT = 15 # seconds

class MorseTestRunner(unittest.TextTestRunner):
        
    def setup_logging(self):
        logger = logging.getLogger('morsetesting')
        logger.setLevel(logging.DEBUG)

        ch = logging.StreamHandler()
        ch.setLevel(logging.DEBUG)

        formatter = logging.Formatter('[%(asctime)s (%(levelname)s)]   %(message)s')

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


    def setUpMw(self):
        """ This method can be overloaded by subclasses to define
        environment setup, before the launching of the Morse environment
        pass
        """
        pass


    def setUp(self):
        
        testlogger.info("Starting test " + self.id())

        self.logfile_name = self.__class__.__name__ + ".log"
        # Wait for a second
        #  to wait for ports open in previous tests to be closed
        sleep(1)
        
        self.morse_initialized = False
        self.setUpMw()
        self.startmorse(self)

    def tearDownMw(self):
        """ This method can be overloaded by subclasses to clean up
        environment setup
        """
        pass
    
    def tearDown(self):
        self.stopmorse()
        self.tearDownMw()
        
    @abstractmethod
    def setUpEnv(self):
        """ This method must be overloaded by subclasses to define a
        simulation environment.

        The code must follow the :doc:`Builder API <morse/dev/builder>`
        convention (without the import of the `morsebuilder` module which
        is automatically added).
        """
        pass

    def wait_initialization(self):
        testlogger.info("Waiting for MORSE to initialize... (timeout: %s sec)" % \
                        BLENDER_INITIALIZATION_TIMEOUT)
        with open(self.logfile_name) as log:
            line = ""
            while not "SCENE INITIALIZED" in line:
                line  = log.readline()
                if "INITIALIZATION ERROR" in line:
                    testlogger.error("Error during MORSE initialization! Check "
                                     "the log file.")
                    return
            
            self.morse_initialized = True

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

            self.logfile = open(self.logfile_name, 'w')
            self.morse_process = subprocess.Popen([cmd, 'run', temp_builder_script], stdout=self.logfile, stderr=subprocess.STDOUT)
        except OSError as ose:
            testlogger.error("Error while launching MORSE! Check you can run it from command-line\n" + \
                    " and if you use the $MORSE_ROOT env variable, check it points to a correct " + \
                    " place!")
            raise ose
        
        morse_initialized = False

        t = threading.Thread(target=self.wait_initialization)
        t.start()
        t.join(BLENDER_INITIALIZATION_TIMEOUT)
        
        if self.morse_initialized:
            self.pid = self.morse_process.pid
            testlogger.info("MORSE successfully initialized with PID %s" % self.pid)
        else:
            self.morse_process.terminate()
            raise MorseTestingError("MORSE did not start successfully! Check %s "
                                    "for details." % self.logfile_name)
    
    def stopmorse(self):
        """ Cleanly stop MORSE
        """
        import socket
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(("localhost", 4000))
        s.send(b"id1 simulation quit\n")

        with open(self.logfile_name) as log:
            line = ""
            while not "EXITING SIMULATION" in line:
                line  = log.readline()
        

        self.morse_process.terminate()
        testlogger.info("MORSE stopped")
    
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
