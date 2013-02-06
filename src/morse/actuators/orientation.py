import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.actuator
from morse.helpers.components import add_data
from morse.core import mathutils

class Orientation(morse.core.actuator.Actuator):
    """
    Motion controller changing immediately the robot orientation.

    This actuator reads the values of angles of rotation around the 3
    axis and applies them to the associated robot. This rotation is
    applied instantly (not in a realist way). Angles are expected in
    radians.
    """

    _name = "Orientation Actuator"
    _short_desc = "An actuator to change instantly robot orientation."

    add_data('yaw', 'Initial robot yaw', "float",
                    'Rotation of the robot around Z axis, in radian')
    add_data('pitch', 'Initial robot pitch', "float",
                      'Rotation of the robot around Y axis, in radian')
    add_data('roll', 'Initial robot roll', "float",
                      'Rotation of the robot around X axis, in radian')

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        self.orientation = self.bge_object.orientation.to_euler('XYZ')

        self.local_data['yaw'] = self.orientation.z
        self.local_data['pitch'] = self.orientation.y
        self.local_data['roll'] = self.orientation.x

        logger.info('Component initialized')

    def default_action(self):
        """ Change the parent robot orientation. """
        # Get the Blender object of the parent robot
        parent = self.robot_parent.bge_object

        # New parent orientation
        orientation = mathutils.Euler([self.local_data['roll'], \
                                       self.local_data['pitch'], \
                                       self.local_data['yaw']])

        # Suspend Bullet physics engine, which doesnt like teleport
        # or instant rotation done by the Orientation actuator (avoid tilt)
        parent.suspendDynamics()
        parent.worldOrientation = orientation.to_matrix()
        parent.restoreDynamics()
