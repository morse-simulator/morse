import logging; logger = logging.getLogger("morse." + __name__)
import GameLogic
import morse.core.actuator

class OrientationActuatorClass(morse.core.actuator.MorseActuatorClass):
    """ Motion controller changing the robot orientation

    This class will read angles as input from an external middleware,
    and then change the robot orientation accordingly.
    """

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self.local_data['rx'] = 0.0
        self.local_data['ry'] = 0.0
        self.local_data['rz'] = 0.0

        logger.info('Component initialized')

    def default_action(self):
        """ Change the parent robot orientation. """
        # Get the Blender object of the parent robot
        parent = self.robot_parent.blender_obj
        # Change the parent orientation
        parent.orientation = [self.local_data['rx'], 
                              self.local_data['ry'], 
                              self.local_data['rz']]

