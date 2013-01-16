import logging; logger = logging.getLogger("morse." + __name__)
import mathutils
import morse.core.actuator
from morse.core.services import service

class TeleportActuatorClass(morse.core.actuator.Actuator):
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

    @service
    def translate(self, x, y = 0.0, z=0.0):
        """ Translate the actuator owner by the given (x,y,z) vector.

        This is a **relative** displacement.

        :param x: X translation, in meters
        :param y: (default: 0.0) Y translation, in meters
        :param z: (default: 0.0) Z translation, in meters
        """
        self.local_data['x'] += x
        self.local_data['y'] += y
        self.local_data['z'] += z

    @service
    def rotate(self, roll, pitch = 0.0, yaw=0.0):
        """ Rotates the actuator owner by the given (roll,pitch,yaw).

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
        parent = self.robot_parent.blender_obj

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
