import logging; logger = logging.getLogger("morse." + __name__)

from morse.modifiers.abstract_modifier import AbstractModifier
from math import cos, sin

class OdometryNoiseModifier(AbstractModifier):
    """
    This modifier allows to simulate two common issues when calculating odometry :

    - an error in the scale factor used to compute the distance from the value
      returned by the odometer (parameter **factor**)
    - the gyroscope natural drift (parameter **gyro_drift** (rad by tick))
    
    Modified data
    -------------

    The modifier only accumulate errors for a 2D odometry sensor. It modifies so
    the following variables :

    - **dS** by the scale factor
    - **dx** considering the scale factor and gyroscope drift
    - **dy** considering the scale factor and gyroscope drift
    - **dyaw** considering the gyroscope drift
    - **x** considering the scale factor and gyroscope drift
    - **y** considering the scale factor and gyroscope drift
    - **yaw** considering the gyroscope drift
    - **vx** considering the new **dx**
    - **vy** considering the new **dy**
    - **wz** considering the new **dyaw**

    Available methods
    -----------------

    - ``noisify``: Simulate drift of gyroscope and possible error in the scale
      factor
    """
    
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

