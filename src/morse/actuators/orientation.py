import logging; logger = logging.getLogger("morse." + __name__)
import bge
import mathutils
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

        self.orientation = self.blender_obj.orientation.to_euler('XYZ')

        self.local_data['yaw'] = self.orientation.z
        self.local_data['pitch'] = self.orientation.y
        self.local_data['roll'] = self.orientation.x

        logger.info('Component initialized')

    def default_action(self):
        """ Change the parent robot orientation. """
        # Get the Blender object of the parent robot
        parent = self.robot_parent.blender_obj
        # Change the parent orientation
        rot = mathutils.Euler([self.local_data['roll'], 
                              self.local_data['pitch'], 
                              self.local_data['yaw']])
        parent.orientation = rot.to_matrix()

