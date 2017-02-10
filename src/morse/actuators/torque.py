import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.services import service
import GameLogic
import morse.core.actuator

class TActuatorClass(morse.core.actuator.MorseActuatorClass):
    """ Motion controller using torque.

    This class will read torque (T)
    as input from an external middleware, and then apply it
    to the parent object (for example wheel).
    """

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self.local_data['torque'] = 0.0

        logger.info('Component initialized')

    @service
    def set_torque(self, t):
        self.local_data['torque'] = t

    @service
    def stop(self):
        self.local_data['torque'] = 0.0

    def default_action(self):
        """ Apply (t) to the parent object. """
             
        # Get the Blender object of the parent robot
        parent = self.robot_parent.blender_obj

        # Give the movement instructions directly to the parent
        # The second parameter specifies a "local" movement
        parent.applyTorque([0, 0, self.local_data['torque']], True)
