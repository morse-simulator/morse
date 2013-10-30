"""
This module deals with the time management in Morse, providing several
possible implementations.

At the moment, it provides two implementations:
    - best effort (i.e. try to simulate at real-time, by dropping
      frame). The simulation is less acurate, because the physical steps
      are not really constant.
    - fixed simulation step. Compute all physical / logical step. The
      simulation will be more precise, but the simulation time will
      differ from computer clock time.
"""

import logging
logger = logging.getLogger("morse." + __name__)
import time
from morse.core import blenderapi

class BestEffort:
    def __init__ (self):
        self.time = time.time()
        logger.info('Morse configured in Best Effort Mode')

    def update (self):
        self.time = time.time()

class FixedSimulationStep:
    def __init__ (self):
        self.time = time.time()
        self._incr = 1.0 / blenderapi.getfrequency() * 1000.0

        logger.info('Morse configured in Fixed Simulation Step Mode with '
                     'time step of %f sec ( 1.0 /  %d)' %
                     (self._incr / 1000.0, 
                      blenderapi.getfrequency()))

    def update (self):
        self.time = self.time + self._incr


