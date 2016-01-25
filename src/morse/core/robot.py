import logging; logger = logging.getLogger("morse." + __name__)
from abc import ABCMeta
import morse.core.object
from morse.core import blenderapi
from morse.core import mathutils
from morse.helpers.components import add_property

class Robot(morse.core.object.Object):
    """ Basic Class for all robots

    Inherits from the base object class.
    """
    add_property('_no_gravity', False, 'NoGravity', 'bool',
                'Indicate if we want to consider the gravity for this \
                robot If true, the behaviour is less realistic as the \
                simulator will automatically compensate it. This setting \
                is useful for non-realistic model flying or submarine \
                robot ')
    add_property('_is_ground_robot', False, 'GroundRobot', 'bool',
                 'Indicate if the robot is a ground robot, i.e. \
                  basically if it has no way to control its position on the \
                  Z axis, nor this X and Y rotation axis')

    # Make this an abstract class
    __metaclass__ = ABCMeta

    def __init__ (self, obj, parent=None):
        """ Constructor method. """
        # Call the constructor of the parent class
        morse.core.object.Object.__init__(self, obj, parent)
        
        # Add the variable move_status to the object
        self.move_status = "Stop"

        # shift against the simulator time (in ms)
        self.time_shift = 0.0

        self.is_dynamic = bool(self.bge_object.getPhysicsId())

    def action(self):
        """ Call the regular action function of the component. """

        if not self.periodic_call():
            return

        # Update the component's position in the world
        self.position_3d.update(self.bge_object)

        self.default_action()

    def gettime(self):
        """ Return the current time, as seen by the robot, in seconds """
        return blenderapi.persistantstorage().time.time + self.time_shift

    def apply_speed(self, kind, linear_speed, angular_speed):
        """
        Apply speed parameter to the robot

        :param string kind: the kind of control to apply. Can be one of
        ['Position', 'Velocity'].
        :param list linear_speed: the list of linear speed to apply, for
        each axis, in m/s.
        :param list angular_speed: the list of angular speed to apply,
        for each axis, in rad/s.
        """

        parent = self.bge_object
        must_fight_against_gravity = self.is_dynamic and self._no_gravity

        if must_fight_against_gravity:
            parent.applyForce(-blenderapi.gravity())

        if kind == 'Position':
            if must_fight_against_gravity:
                parent.worldLinearVelocity = [0.0, 0.0, 0.0]
            parent.applyMovement(linear_speed, True)
            parent.applyRotation(angular_speed, True)
        elif kind == 'Velocity':
            if self._is_ground_robot:
                """
                A ground robot cannot control its vz not rx, ry speed in
                the world frame. So convert {linear, angular}_speed in
                world frame, remove uncontrolable part and then pass it
                against in the robot frame"""
                linear_speed = mathutils.Vector(linear_speed)
                angular_speed = mathutils.Vector(angular_speed)
                linear_speed.rotate(parent.worldOrientation)
                angular_speed.rotate(parent.worldOrientation)
                linear_speed[2] = parent.worldLinearVelocity[2]
                angular_speed[0] = parent.worldAngularVelocity[0]
                angular_speed[1] = parent.worldAngularVelocity[1]
                linear_speed.rotate(parent.worldOrientation.transposed())
                angular_speed.rotate(parent.worldOrientation.transposed())

            # Workaround against 'strange behaviour' for robot with
            # 'Dynamic' Physics Controller. [0.0, 0.0, 0.0] seems to be
            # considered in a special way, i.e. is basically ignored.
            # Setting it to 1e-6 instead of 0.0 seems to do the trick!
            if angular_speed == [0.0, 0.0, 0.0]:
                angular_speed = [0.0, 0.0, 1e-6]
            parent.setLinearVelocity(linear_speed, True)
            parent.setAngularVelocity(angular_speed, True)

    def force_pose(self, position, orientation):
        parent = self.bge_object

        if self.is_dynamic:
            parent.applyForce(-blenderapi.gravity())
            parent.worldLinearVelocity = [0.0, 0.0, 0.0]
            parent.suspendDynamics()

        if position:
            parent.worldPosition = position

        if orientation:
            parent.worldOrientation = orientation

        if self.is_dynamic:
            parent.restoreDynamics()
