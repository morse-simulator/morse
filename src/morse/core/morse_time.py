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

class Stats:
    def __init__(self):
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0

    def update(self, x):
        self.n = self.n + 1
        delta = x - self.mean
        self.mean = self.mean + delta/self.n
        self.m2 = self.m2 + delta * (x - self.mean)

    @property
    def variance(self):
        return self.m2 / (self.n - 1)

class BestEffort:
    def __init__ (self):
        self.time = time.time()

        self._stat_jitter = Stats()
        self._stat_nb_frame = Stats()
        self._time_frame = 0.0
        self._last_time = 0.0
        self._nb_frame = 0

        logger.info('Morse configured in Best Effort Mode')

    def update (self):
        self.time = time.time()
        self._update_statistics()

    def statistics (self):
        return {
            "mean_time" : self._stat_jitter.mean,
            "variance_time": self._stat_jitter.variance,
            "mean_frame_by_sec": self._stat_nb_frame.mean,
            "variance_frame_by_sec": self._stat_nb_frame.variance
        }

    def _update_statistics(self):
        if self._last_time == 0.0:
            self._last_time = self.time
        else:
            ds = self.time - self._last_time
            self._last_time = self.time
            self._stat_jitter.update(ds)

        if self._nb_frame == 0:
            self._time_frame = self.time
            self._nb_frame = 1
        else:
            if self.time - self._time_frame > 1000.0:
                self._stat_nb_frame.update(self._nb_frame)
                self._nb_frame = 0
            else:
                self._nb_frame = self._nb_frame + 1

class FixedSimulationStep:
    def __init__ (self):
        self.time = time.time()
        self._incr = 1.0 / blenderapi.getfrequency() * 1000.0

        self._stat_jitter = Stats()
        self._last_time = 0.0

        logger.info('Morse configured in Fixed Simulation Step Mode with '
                     'time step of %f sec ( 1.0 /  %d)' %
                     (self._incr / 1000.0, 
                      blenderapi.getfrequency()))

    def update (self):
        self.time = self.time + self._incr
        self._update_statistics()

    def statistics (self):
        return {
            "mean_time" : self._stat_jitter.mean,
            "variance_time": self._stat_jitter.variance,
            "diff_real_time": self.time - time.time()
        }

    def _update_statistics(self):
        if self._last_time == 0.0:
            self._last_time = time.time()
        else:
            ds = time.time() - self._last_time
            self._last_time = time.time()
            self._stat_jitter.update(ds)
