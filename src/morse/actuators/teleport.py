import logging; logger = logging.getLogger("morse." + __name__)
import mathutils
import morse.core.actuator

class TeleportActuatorClass(morse.core.actuator.MorseActuatorClass):
    """ Motion controller changing the robot pose (position and orientation)

    This class will read a position vector and orientation quaternion as input
    from an external middleware, and then change the robot pose accordingly.
    """

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        orientation = self.blender_obj.worldOrientation.to_euler('XYZ')
        position = self.blender_obj.worldPosition

        self.local_data['x'] = position.x
        self.local_data['y'] = position.y
        self.local_data['z'] = position.z
        self.local_data['roll'] = orientation.x
        self.local_data['pitch'] = orientation.y
        self.local_data['yaw'] = orientation.z

        logger.info('Component initialized')

    def default_action(self):
        """ Change the parent robot pose. """
        # Get the Blender object of the parent robot
        parent = self.robot_parent.blender_obj
        # Change the parent position
        parent.worldPosition = mathutils.Vector((self.local_data['x'],
                                                 self.local_data['y'],
                                                 self.local_data['z']))
        # Change the parent orientation
        rot = mathutils.Euler([self.local_data['roll'], 
                               self.local_data['pitch'], 
                               self.local_data['yaw']])
        parent.worldOrientation = rot.to_matrix()

