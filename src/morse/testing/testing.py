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
import signal

from morse.testing.exceptions import MorseTestingError
from morse.core.morse_time import TimeStrategies

BLENDER_INITIALIZATION_TIMEOUT = 15 # seconds

MODE_INDEX = 0
CURRENT_TIME_MODE = None
ALL_TIME_MODES = None
INITIALIZED_LOGGER = False

class MorseTestRunner(unittest.TextTestRunner):

        
    def setup_logging(self):
        global INITIALIZED_LOGGER
        logger = logging.getLogger('morsetesting')
        logger.setLevel(logging.DEBUG)

        ch = logging.StreamHandler()
        ch.setLevel(logging.DEBUG)

        formatter = logging.Formatter('[%(asctime)s (%(levelname)s)]   %(message)s')

        ch.setFormatter(formatter)

        logger.addHandler(ch)
        INITIALIZED_LOGGER = True

    def run(self, suite):
        global INITIALIZED_LOGGER
        if not INITIALIZED_LOGGER:
            self.setup_logging()
        return unittest.TextTestRunner.run(self, suite)

def follow(file):
    """ Really emulate tail -f

    See http://stackoverflow.com/questions/1475950/tail-f-in-python-with-no-time-sleep
    for a detailled discussion on the subject
    """
    while True:
        line = file.readline()
        if not line:
            sleep(0.1)    # Sleep briefly
            continue
        yield line

class MorseSwitchTimeMode(unittest.TestCase):
    def test_switch(self):
        global ALL_TIME_MODES
        global CURRENT_TIME_MODE
        global MODE_INDEX
        CURRENT_TIME_MODE = ALL_TIME_MODES[MODE_INDEX]
        MODE_INDEX += 1

class MorseTestCase(unittest.TestCase):

    # Make this an abstract class
    __metaclass__ = ABCMeta


    def setUpMw(self):
        """ This method can be overloaded by subclasses to define
        environment setup, before the launching of the Morse environment
        pass
        """
        pass

    def _checkMorseException(self):
        """ Check in the Morse output if some python error happens"""

        with open(self.logfile_name) as log:
            lines = follow(log)
            for line in lines:
                # Python Error Case
                if "[ERROR][MORSE]" in line:
                    testlogger.error(line.strip())
                    testlogger.error("Exception detected in Morse execution : "
                                     "see %s for details."
                                     " Exiting the current test !" % self.logfile_name)
                    os.kill(os.getpid(), signal.SIGINT)
                    return

                # End of simulation, exit the thread
                if "EXITING SIMULATION" in line:
                    return

    def setUp(self):
        
        testlogger.info("Starting test " + self.id() + " in " + TimeStrategies.human_repr(CURRENT_TIME_MODE))

        self.logfile_name = self.__class__.__name__ + ".log"
        # Wait for a second
        #  to wait for ports open in previous tests to be closed
        sleep(1)

        self.morse_initialized = False
        self.setUpMw()
        self.startmorse(self)
        self.t = threading.Thread(target=self._checkMorseException)
        self.t.start()

    def tearDownMw(self):
        """ This method can be overloaded by subclasses to clean up
        environment setup
        """
        pass
    
    def tearDown(self):
        self.stopmorse()
        self.tearDownMw()
        self.logfile.close() # force to flush
        self.t.join()

        
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
        """ Wait until Morse is initialized """

        testlogger.info("Waiting for MORSE to initialize... (timeout: %s sec)" % \
                        BLENDER_INITIALIZATION_TIMEOUT)
        with open(self.logfile_name) as log:
            lines = follow(log)
            for line in lines:
                if  ("[ERROR][MORSE]" in line) or ("INITIALIZATION ERROR" in line):
                    testlogger.error("Error during MORSE initialization! Check "
                                     "the log file.")
                    return
                if "SCENE INITIALIZED" in line:
                    self.morse_initialized = True
                    return

    def run(self, result=None):
        """ Overwrite unittest.TestCase::run

        Detect KeyBoardInterrupt exception , due to user or a SIGINIT In
        particular, it can happen if we detect an exception in the Morse
        execution. In this case, clean up correctly the environnement.
        """
       
        try:
            return unittest.TestCase.run(self, result)
        except KeyboardInterrupt as e:
            self.tearDownMw()
            if self.pid:
                os.kill(self.pid, signal.SIGKILL)
            if result:
                result.addError(self, sys.exc_info())

    def _extract_pid(self):
        """ Extract the pid from the log file.

        We can not simply rely on Popen.subprocess.pid because we need the PID of the
        Blender process itself, not the (Python) MORSE process.
        """

        with open(self.logfile_name) as log:
            for line in log:
                if "PID" in line:
                    words = line.split()
                    return int(words[-1])


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

        t = threading.Thread(target=self.wait_initialization)
        t.start()
        t.join(BLENDER_INITIALIZATION_TIMEOUT)
        
        if self.morse_initialized:
            self.pid = self._extract_pid()
            testlogger.info("MORSE successfully initialized with PID %s" % self.pid)
        else:
            self.morse_process.terminate()
            raise MorseTestingError("MORSE did not start successfully! Check %s "
                                    "for details." % self.logfile_name)
    
    def stopmorse(self):
        """ Cleanly stop MORSE
        """
        import socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.connect(("localhost", 4000))
            sock.send(b"id1 simulation quit\n")
        except (socket.error, KeyboardInterrupt):
            sock.close()
            sock = None
            testlogger.info("MORSE crashed")

        if sock:
            sock.close()
            with open(self.logfile_name) as log:
                lines = follow(log)
                for line in lines:
                    if "EXITING SIMULATION" in line:
                        return

        if self.pid:
            os.kill(self.pid, signal.SIGKILL)
        testlogger.info("MORSE stopped")
    
    def generate_builder_script(self, test_case):
        
        tmp_name = ""
        # We need to generate a temp builder file in case of running
        # several test cases with different environment:
        # Blender must be restarted and called again with the right
        # environment.
        with tempfile.NamedTemporaryFile(delete = False) as tmp:
            tmp.write(b"from morse.builder import *\n")
            tmp.write(b"from morse.builder.actuators import *\n")
            tmp.write(b"from morse.builder.sensors import *\n")
            tmp.write(b"from morse.builder.blenderobjects import *\n")
            tmp.write(b"class MyEnv():\n")
            tmp.write(inspect.getsource(test_case.setUpEnv).encode())
            tmp.write(b"        env.set_time_strategy(")
            tmp.write(TimeStrategies.python_repr(CURRENT_TIME_MODE))
            tmp.write(b")\n")
            tmp.write(b"MyEnv().setUpEnv()\n")
            tmp.flush()
            tmp_name = tmp.name
        
        testlogger.info("Created a temporary builder file for test-case " +\
            test_case.__class__.__name__)
        return tmp_name


class MorseMoveTestCase(MorseTestCase):
    """ This subclass of MorseTestCase can be used to check for moving
    actuator, basically by testing a complete pose

    This class assumes lot of stuff to work properly:
        - the tested robot is called 'robot'
        - the pose sensor is called 'robot.pose'
        - if assertAlmostEqualPositionThenFix,  it assumes there is a
            - a 'robot.motion' actuator (basically what we test)
            - a 'robot.teleport' actuator (to move to a specific
              situation)
    """

    def assertAlmostEqualPosition(self, morse, tested_pos, precision):
        """ 
        Test against a position, presented as an array of 6 double (x,
        y, z, yaw, pitch, roll)
        """
        pose = morse.robot.pose.get()
        self.assertAlmostEqual(pose['x'], tested_pos[0], delta=precision)
        self.assertAlmostEqual(pose['y'], tested_pos[1], delta=precision)
        self.assertAlmostEqual(pose['z'], tested_pos[2], delta=precision)
        self.assertAlmostEqual(pose['yaw'], tested_pos[3], delta=precision)
        self.assertAlmostEqual(pose['pitch'], tested_pos[4], delta=precision)
        self.assertAlmostEqual(pose['roll'], tested_pos[5], delta=precision)

    def assertAlmostEqualPositionThenFix(self, morse, tested_pos, precision):
        """
        Test against a position, presented as an array of 6 double (x,
        y, z, yaw, pitch, roll). When done, move the robot to this
        specific position. It allows to reduce the cumulated error
        between the different part of the test
        """
        self.assertAlmostEqualPosition(morse, tested_pos, precision)

        morse.deactivate('robot.motion')
        morse.activate('robot.teleport')
        # +0.01  on z to be a bit higher than the ground
        morse.robot.teleport.publish(
                {'x': tested_pos[0],
                 'y': tested_pos[1],
                 'z': tested_pos[2] + 0.01, 
                 'yaw': tested_pos[3],
                 'pitch': tested_pos[4],
                 'roll': tested_pos[5]})
        morse.sleep(0.1)
        morse.deactivate('robot.teleport')
        morse.activate('robot.motion')
        morse.sleep(0.1)


class MorseBuilderFailureTestCase(MorseTestCase):
    """ This subclass of MorseTestCase can be used to test MORSE handles
    properly ill-constructed Builder scripts.

    It will *fail* if the Blender Game Engine get started.
    """

    # Make this an abstract class
    __metaclass__ = ABCMeta

    def wait_initialization(self):
        """ Wait until Morse is initialized """

        testlogger.info("Waiting for MORSE to parse the scene... (timeout: %s sec)" % \
                        BLENDER_INITIALIZATION_TIMEOUT)
        # we assume we will correctly detect Builder script issue, so wait_initialization
        # 'succeed'.
        self.morse_initialized = True

        with open(self.logfile_name) as log:
            lines = follow(log)
            for line in lines:
                if "Blender Game Engine Started" in line:
                    testlogger.error("Blender Game Engine started!"
                                     " This is not expected."
                                     " See %s for details." % self.logfile_name)
                    os.kill(os.getpid(), signal.SIGINT)
                    return
                elif "[ERROR][MORSE]" in line:
                    # use 'info' since we suppose to get this error
                    testlogger.info("MORSE initialization error: %s"%line.strip())
                    return

    def _checkMorseException(self):
        return


def main(*args, **kwargs):
    test_cases = args
    time_modes = kwargs.get('time_modes', [TimeStrategies.BestEffort, TimeStrategies.FixedSimulationStep])
    import sys
    if sys.argv[0].endswith('blender'):
        # If we arrive here from within MORSE, we have probably run
        # morse [exec|run] my_test.py
        # If this case, simply build the environment based on the
        # setUpEnv of the first test.
        for test_class in test_cases:
            test_class().setUpEnv()
            return
    import unittest
    suite = unittest.TestSuite()
    loader = unittest.TestLoader()
    global ALL_TIME_MODES
    ALL_TIME_MODES = time_modes

    tests = None
    switch = loader.loadTestsFromTestCase(MorseSwitchTimeMode)
    for test_class in test_cases:
        tests = loader.loadTestsFromTestCase(test_class)
    for time_mode in time_modes:
        suite.addTests(switch)
        suite.addTests(tests)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())
