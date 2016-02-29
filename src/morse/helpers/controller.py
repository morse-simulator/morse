""" 
A collection of simple controllers

For now, implement only a simple PID controller
"""
import logging
logger = logging.getLogger("morse." + __name__)
from morse.core import blenderapi

def clamp(n, smallest, largest): 
    return max(smallest, min(n, largest))

class PIDController(object):
    def __init__(self, kp=1.0, kd=1.0, ki=1.0, limits_integrator = 10.0):
        self.kp = kp
        self.kd = kd
        self.ki = ki

        self._setpoint = 0.0

        self._last_time = 0.0
        self._last_error = 0.0

        self._integrator = 0.0
        self._limits_integrator = limits_integrator

    @property
    def setpoint(self):
        return self._setpoint

    @setpoint.setter
    def setpoint(self, new_setpoint):
        if (abs(self._setpoint - new_setpoint) > 0.1):
            self.reset()
        self._setpoint = new_setpoint

    def reset(self):
        self.last_error = 0.0
        self._last_time = 0.0
        self._integrator = 0.0

    def update(self, measured_value):
        error = self._setpoint - measured_value
        current_time = blenderapi.persistantstorage().time.time

        # Calculate the derivative of the error
        timediff = current_time - self._last_time
        if timediff > 0:
            error_diff = (error - self._last_error) / timediff
        else:
            error_diff = 0.0

        # Remember values for next time
        self._last_error = error
        self._last_time = current_time

        self._integrator += error
        self._integrator = clamp(self._integrator,
                                 -self._limits_integrator,
                                  self._limits_integrator)

        return self.kp * error + self.kd * error_diff + self.ki * self._integrator
