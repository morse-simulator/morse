import logging; logger = logging.getLogger("morse." + __name__)

from morse.modifiers.abstract_modifier import AbstractModifier
from math import cos, sin

class OdometryNoiseModifier(AbstractModifier):
    """ Apply a gaussian noise to odometry data """
    
    def initialize(self):
        self._factor = float(self.parameter("factor", default=1.05))
        self._gyro_drift = float(self.parameter("gyro_drift", default=0))
        self._drift_x = 0.0
        self._drift_y = 0.0
        self._drift_yaw = 0.0

    def modify(self):
        # Basic 2D odometry implementation dx = dS * sin(yaw) and
        #                                  dy = dS * cos(yaw)
        # If we have some error on  dS and yaw, we have
        # dx      = factor * dS * sin(yaw + drift_yaw)
        #         = factor * dS * sin(yaw) * cos(drift_yaw) +
        #           factor * dS * cos(yaw)  * sin(drift_yaw)
        #         = factor * ( dx * cos(drift_yaw) +  dy * sin(drift_yaw))
        # Same thing to compute dy
        try:
            self._drift_yaw += self._gyro_drift
            dx = self._factor * ( self.data['dx'] * cos(self._drift_yaw) +
                                  self.data['dy'] * sin(self._drift_yaw))
            dy = self._factor * ( self.data['dy'] * cos(self._drift_yaw) -
                                  self.data['dx'] * sin(self._drift_yaw))
            
            self._drift_x +=  dx - self.data['dx']
            self._drift_y +=  dy - self.data['dy']
            
            self.data['dS'] *= self._factor
            self.data['dx'] = dx
            self.data['dy'] = dy
            self.data['dyaw'] += self._gyro_drift
            
            self.data['x'] += self._drift_x
            self.data['y'] += self._drift_y
            self.data['yaw'] += self._drift_yaw
            
            freq = self.component_instance.frequency
            
            self.data['vx'] = self.data['dx'] / freq
            self.data['vy'] = self.data['dy'] / freq
            self.data['wz'] = self.data['dyaw'] / freq
        
        except KeyError as detail:
            self.key_error(detail)

