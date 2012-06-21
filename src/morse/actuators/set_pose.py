import logging; logger = logging.getLogger("morse." + __name__)
import bge
import mathutils
import morse.core.actuator

class SetPoseActuatorClass(morse.core.actuator.MorseActuatorClass):
    """ Motion controller changing the robot pose (position and orientation)

    This class will read a position vector and orientation quaternion as input
    from an external middleware, and then change the robot pose accordingly.
    """

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        orientation = self.blender_obj.worldOrientation.to_quaternion()
        position = self.blender_obj.worldPosition

        self.local_data['x'] = position.x
        self.local_data['y'] = position.y
        self.local_data['z'] = position.z
        self.local_data['qw'] = orientation.w
        self.local_data['qx'] = orientation.x
        self.local_data['qy'] = orientation.y
        self.local_data['qz'] = orientation.z

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
        rot = mathutils.Quaternion((self.local_data['qw'],
                                    self.local_data['qx'],
                                    self.local_data['qy'],
                                    self.local_data['qz']))
        parent.worldOrientation = rot.to_matrix()

