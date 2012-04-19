import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.wheeled_robot

class SegwayRMP400PhysicsClass(morse.core.wheeled_robot.MorsePhysicsRobotClass):
    """ Class definition for the Segway RMP400 base.
        Sub class of Morse_Object. """
              
    def __init__(self, obj, parent=None):
        """ Constructor method.
            Receives the reference to the Blender object.
            Optionally it gets the name of the object's parent,
            but that information is not currently used for a robot. """
        logger.info('%s initialization' % obj.name)

        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        # XXX: Hack to make the robot turn at the expected speed
        #  using the v_omega_differential_drive actuator
        # Real distance between the wheel objects in Blender:
        #self._trackWidth = 0.624
        # Best working when using this distance, obtained by comparing the
        #  trayectories followed by this robot with those of the ATRV with
        #  the regular v_omega actuator
        if obj['FixTurningSpeed'] != 0:
            self._trackWidth = obj['FixTurningSpeed']
            logger.warn("Using wheel separation of %.4f" % self._trackWidth)
            #self._trackWidth = 1.23
            #self._trackWidth = 1.248
            #self._trackWidth = 1.425

        logger.info('Component initialized')


    def default_action(self):
        """ Main function of this component. """
        pass
