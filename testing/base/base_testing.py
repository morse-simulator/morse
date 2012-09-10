#! /usr/bin/env python
"""
This script tests some of the base functionalities of MORSE.
"""

import sys
from morse.testing.testing import MorseTestCase

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

class BaseTest(MorseTestCase):

    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        
        # Adding 2 robots
        robot1 = Robot('jido')        
        robot2 = Robot('atrv')
        
        env = Environment('indoors-1/indoor-1')

    def test_list_robots(self):
        """ Tests the simulator can return the list of robots
        
        This test is guaranteed to be started only when the simulator
        is ready.
        """
        
        # Initialize a socket connection to the simulator
        import socket
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(("localhost", 4000))
        sockf = s.makefile()
        
        # Queries for the list of robots
        s.send(b"id1 simulation list_robots\n")
        
        result = sockf.readline()
        id, success, robots = result.strip().split(' ', 2)
        self.assertEquals(success, "SUCCESS")
        
        import ast
        robotsset = set(ast.literal_eval(robots))
        self.assertEquals(robotsset, {'Jido', 'ATRV'})
        sockf.close()
        s.close()

    def test_socket_request_parser_resilience(self):
        """ Tests that the socket request parser is resilient to
        useless whitespaces.
        
        """
        
        # Initialize a socket connection to the simulator
        import socket
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(("localhost", 4000))
        sockf = s.makefile()
        
        queries = [b"id1 simulation list_robots\n",
                   b"id1  simulation list_robots\n",
                   b"id1\tsimulation\tlist_robots\n",
                   b"id1 \t simulation \t list_robots\n",
                   b"   id1 simulation list_robots  \n"]

        for q in queries:
            s.send(q)
            result = sockf.readline()
            id, success, robots = result.strip().split(' ', 2)
            self.assertEquals(success, "SUCCESS")

        sockf.close()
        s.close()


########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(BaseTest)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

