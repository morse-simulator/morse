import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.wheeled_robot

class Pioneer3DX(morse.core.wheeled_robot.MorsePhysicsRobot):
    """ 
    This robot uses the Physics Constraints in Blender to allow the wheels to
    behave more realistically. The wheels turn as the robot moves, and they have
    ``Rigid Body`` physics, so that they can also have collisions with nearby
    objects.

    It has two differential drive wheels, and an additional caster wheel for
    stability.  Since the wheels of this robot use the ``Rigid Body`` physics, it
    must be controlled with the :doc:`v_omega_diff_drive
    <../actuators/v_omega_diff_drive>` actuator.
    """

    _name = 'Pionner 3-DX platform'
              
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
