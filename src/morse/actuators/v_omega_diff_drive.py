import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.services import service
import morse.core.actuator
import math

class VWDiffDriveActuatorClass(morse.core.actuator.MorseActuatorClass):
    """ Motion controller using linear and angular speeds

    This class will read linear and angular speeds (V, W)
    as input from an external middleware, and then apply them
    to the parent robot.  Differs from the standard V,W controller
    in that individual wheel speeds are controlled rather than 
    chassis speed
    """

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self.local_data['v'] = 0.0
        self.local_data['w'] = 0.0

        self._stopped=True

        # get track width for calculating wheel speeds from yaw rate
        parent = self.robot_parent
        self._trackWidth = parent._trackWidth
        self._radius = parent._wheelRadius

        logger.info('Component initialized')

    @service
    def set_speed(self, v, w):
        self.local_data['v'] = v
        self.local_data['w'] = w

    @service
    def stop(self):
        self.local_data['v'] = 0.0
        self.local_data['w'] = 0.0		
		
		
    def default_action(self):
        """ Apply (v, w) to the parent robot. """

        # calculate desired wheel speeds and set them
        if (abs(self.local_data['v'])<0.001)and(abs(self.local_data['w'])<0.001):
            # stop the wheel when velocity is below a given threshold
            for index in self.robot_parent._wheel_index:
                self.robot_parent._wheel_joints[index].setParam(9,0,100.0)
            
            self._stopped=True
            pass
        else:
            # this is need to "wake up" the physic objects if they have gone to sleep
			# apply a tiny impulse straight down on the object
            if (self._stopped==True):
                self.robot_parent.blender_obj.applyImpulse(self.robot_parent.blender_obj.position,(0.0,0.1,-0.000001))
            
			# no longer stopped
            self._stopped=False
            
            # left and right wheel speeds in m/s
            v_ws_l=(2*self.local_data['v']-self.local_data['w']*self._trackWidth)/2
            v_ws_r=(2*self.local_data['v']+self.local_data['w']*self._trackWidth)/2

            # convert to angular speeds
            w_ws_l=v_ws_l/self._radius
            w_ws_r=v_ws_r/self._radius
            
            # set wheel speeds - front and rear wheels have the same speed
            # Left side wheels
            self.robot_parent._wheel_joints['FL'].setParam(9,w_ws_l,100.0)
            self.robot_parent._wheel_joints['RL'].setParam(9,w_ws_l,100.0)
            # Right side wheels
            self.robot_parent._wheel_joints['FR'].setParam(9,w_ws_r,100.0)
            self.robot_parent._wheel_joints['RR'].setParam(9,w_ws_r,100.0)

            logger.debug("New speeds set: left=%.4f, right=%.4f" % (w_ws_l, w_ws_r))
