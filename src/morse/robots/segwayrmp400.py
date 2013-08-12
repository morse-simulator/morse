import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.wheeled_robot
from morse.helpers.components import add_property

class SegwayRMP400(morse.core.wheeled_robot.MorsePhysicsRobot):
    """
    Simple definition of the RMP400 platform distributed by Segway.

    This robot uses the Physics Constraints in Blender to allow the wheels to
    behave more realistically. The wheels turn as the robot moves, and they have
    ``Rigid Body`` physics, so that they can also have collisions with nearby
    objects.

    It has four differential drive wheels, with the pairs of wheels on each side
    always moving at the same speed. Since the wheels of this robot use the
    ``Rigid Body`` physics, it must be controlled with the :doc:`v_omega_diff_drive
    <../actuators/v_omega_diff_drive>` actuator.

    """

    _name = 'Segway RMP 400 platform'

    add_property('_fix_turning', 0.0, 'FixTurningSpeed', 'double', 
                'Overwrite the value of the distance between wheels in '
                'the computations of the wheel speeds. This effectively '
                'changes the turning speed of the robot, and can be used '
                'to compensate for the slip of the wheels while turning. '
                'The real distance between wheels in the robot is 0.624m. '
                'By forcing a distance of 1.23m, the robot will turn over '
                'a smaller radius, as would a two wheeled differential '
                'drive robot. If the value 0.0 is used, the real '
                'distance between wheels is used.')

    def __init__(self, obj, parent=None):
        """ Constructor method.
            Receives the reference to the Blender object.
            Optionally it gets the name of the object's parent,
            but that information is not currently used for a robot. """
        logger.info('%s initialization' % obj.name)

        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        if self._fix_turning != 0.0:
            self._trackWidth = self._fix_turning
        logger.warn("Using wheel separation of %.4f" % self._trackWidth)

        logger.info('Component initialized')


    def default_action(self):
        """ Main function of this component. """
        pass
