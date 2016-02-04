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
import copy
from morse.core import blenderapi
from morse.helpers.statistics import Stats


class BestEffortStrategy:
    def __init__ (self, relative_time):
        if relative_time:
            self.time = 0.0
            self._real_time_offset = time.time()
        else:
            self.time = time.time()
            self._real_time_offset = 0.0
        self._time_offset = copy.copy(self.time)

        self._stat_jitter = Stats()
        self._stat_nb_frame = Stats()
        self._time_frame = 0.0
        self._last_time = 0.0
        self._nb_frame = 0

        scene = blenderapi.scene()
        for obj in scene.objects:
            if obj.name == '__morse_dt_analyser':
                self._morse_dt_analyser = obj

        self._prepare_compute_dt()

        logger.info('Morse configured in Best Effort Mode')

    def update (self):
        """
        The exact physical time elapsed between two logic call is hard
        to guess. In the nominal case, it is easy, as long as you have
        one logical step per render step. In other case, it depends on 
        logic_max_step, physics_max_step, and "complex" internal logic.

        So, instead of guessing it, observe it. Assuming the physical engine
        is perfect, put a solid at one meter by sec on x axis, and observe
        its displacement between two frame. We have:
            dx = vx * dt
        where vw = 1.0. So we have dx = dt.

        Modern version of Blender (>= 2.77) provides the method
        bge.logic.getFrameTime() so if this information is available,
        just use it.
        """
        current_time = blenderapi.frame_time()
        if current_time == -1:
            self._dt = self._morse_dt_analyser.worldPosition[0] - self.px
            self.time += self._dt
            self._prepare_compute_dt()
        else:
            self.time = current_time + self._time_offset
        self._update_statistics()

    def name(self):
        return 'Best Effort'
    
    @property
    def mean(self):
        return self._stat_jitter.mean

    def _prepare_compute_dt(self):
        self.px = self._morse_dt_analyser.worldPosition[0]
        self._morse_dt_analyser.setLinearVelocity([1.0, 0.0, 0.0], True)

    def statistics(self):
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
            if self.time - self._time_frame > 1.0:
                self._stat_nb_frame.update(self._nb_frame)
                self._nb_frame = 0
            else:
                self._nb_frame = self._nb_frame + 1

    @property
    def real_time(self):
        return time.time() - self._real_time_offset

class FixedSimulationStepStrategy:
    def __init__ (self, relative_time):
        if relative_time:
            self.time = 0.0
            self._real_time_offset = time.time()
        else:
            self.time = time.time()
            self._real_time_offset = 0.0
        self._time_offset = copy.copy(self.time)
        self._incr = 1.0 / blenderapi.getfrequency()

        self._stat_jitter = Stats()
        self._last_time = 0.0

        logger.info('Morse configured in Fixed Simulation Step Mode with '
                    'time step of %f sec ( 1.0 /  %d)' %
                    (self._incr, blenderapi.getfrequency()))

    def update (self):
        current_time = blenderapi.frame_time()
        if current_time == -1:
            self.time = self.time + self._incr
        else:
            self.time = current_time + self._time_offset
        self._update_statistics()

    def name (self):
        return 'Fixed Simulation Step'

    @property
    def real_time(self):
        return time.time() - self._real_time_offset

    @property
    def mean(self):
        return self._incr

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

class TimeStrategies:
    (BestEffort, FixedSimulationStep) = range(2)

    internal_mapping = {
        BestEffort:
            { "impl": BestEffortStrategy,
              "python_repr": b"TimeStrategies.BestEffort",
              "human_repr" : "Best Effort"
            },
        FixedSimulationStep:
            { "impl": FixedSimulationStepStrategy,
              "python_repr": b"TimeStrategies.FixedSimulationStep",
              "human_repr": "Fixed Simulation Step"
            }
        }

    @staticmethod
    def make(strategy, use_relative_time):
        try:
            return TimeStrategies.internal_mapping[strategy]["impl"](use_relative_time)
        except KeyError:
            return None
    @staticmethod
    def python_repr(strategy):
        try:
            return TimeStrategies.internal_mapping[strategy]["python_repr"]
        except KeyError:
            return None

    @staticmethod
    def human_repr(strategy):
        try:
            return TimeStrategies.internal_mapping[strategy]["human_repr"]
        except KeyError:
            return None


def time_isafter(t1, t2):
    """ Returns true if t1 > t2 in morse_time. Returns false otherwise """
    return t2 - t1 < blenderapi.persistantstorage().time.mean / 2
