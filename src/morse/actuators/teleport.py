import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.actuator
from morse.core.services import service
from morse.core import mathutils
from morse.helpers.components import add_data

class Teleport(morse.core.actuator.Actuator):
    """ 
    This actuator teleports the robot to the absolute position and
    orientation with respect to the origin of the Blender coordinate
    reference. Angles are expected in radians.

    .. note::
        Coordinates are given with respect to the origin of
        Blender's coordinate axis.
    """

    _name = "Teleport"
    _short_desc = "Motion controller which changes instantly robot pose \
                   (position and orientation)"

    add_data('x', 'initial robot X position', "float",
              "X coordinate of the destination, in meter")
    add_data('y', 'initial robot Y position', "float",
              "Y coordinate of the destination, in meter")
    add_data('z', 'initial robot Z position', "float",
             "Z coordinate of the destination, in meter")
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

        orientation = self.bge_object.worldOrientation.to_euler('XYZ')
        position = self.bge_object.worldPosition

        self.local_data['x'] = position.x
        self.local_data['y'] = position.y
        self.local_data['z'] = position.z
        self.local_data['roll'] = orientation.x
        self.local_data['pitch'] = orientation.y
        self.local_data['yaw'] = orientation.z

        logger.info('Component initialized')

    @service
    def translate(self, x, y = 0.0, z = 0.0):
        """
        Translate the actuator owner by the given (x,y,z) vector.

        This is a **relative** displacement.

        :param x: X translation, in meter
        :param y: (default: 0.0) Y translation, in meter
        :param z: (default: 0.0) Z translation, in meter
        """
        self.local_data['x'] += x
        self.local_data['y'] += y
        self.local_data['z'] += z

    @service
    def rotate(self, roll, pitch = 0.0, yaw=0.0):
        """
        Rotates the actuator owner by the given (roll,pitch,yaw).

        This is a **relative** rotation.

        :param roll: roll rotation, in radians
        :param pitch: (default: 0.0) pitch rotation, in radians
        :param yaw: (default: 0.0) yaw rotation, in radians
        """
        self.local_data['roll'] += roll
        self.local_data['pitch'] += pitch
        self.local_data['yaw'] += yaw


    def default_action(self):
        """ Change the parent robot pose. """
        # Get the Blender object of the parent robot
        parent = self.robot_parent.bge_object

        # New parent position
        position = mathutils.Vector((self.local_data['x'], \
                                     self.local_data['y'], \
                                     self.local_data['z']))

        # New parent orientation
        orientation = mathutils.Euler([self.local_data['roll'], \
                                       self.local_data['pitch'], \
                                       self.local_data['yaw']])

        # Suspend Bullet physics engine, which doesnt like teleport
        # or instant rotation done by the Orientation actuator (avoid tilt)
        parent.suspendDynamics()
        parent.worldPosition = position
        parent.worldOrientation = orientation.to_matrix()
        parent.restoreDynamics()
