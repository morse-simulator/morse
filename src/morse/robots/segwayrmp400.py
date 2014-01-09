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


    def __init__(self, obj, parent=None):
        """ Constructor method.
            Receives the reference to the Blender object.
            Optionally it gets the name of the object's parent,
            but that information is not currently used for a robot. """
        logger.info('%s initialization' % obj.name)

        # Call the constructor of the parent class
        morse.core.wheeled_robot.MorsePhysicsRobot.__init__(self, obj, parent)

        logger.info('Component initialized')


    def default_action(self):
        """ Main function of this component. """
        pass
