import logging; logger = logging.getLogger("morse." + __name__)
from morse.core import blenderapi
from morse.robots.grasping_robot import GraspingRobot
from morse.core.services import service
from morse.helpers.components import add_property

class Human(GraspingRobot):
    """
    MORSE allows the simulation of humans: you can add a human model in your
    scene, you can control it like any other robot (including from the keyboard
    or via external scripts), and export from your simulation various data like
    the full body pose.

    The human is managed by MORSE as a regular robot, which means it can have
    sensors and actuators attached to it.

    For a general introduction to human-robot interaction simulation with MORSE,
    check the :doc:`HRI main page <../../hri>`.

    """

    _name = "Human avatar"

    add_property('_animations', True, 'Animations', 'bool', "If "
            "true (default), will enable various animations like"
            " the walk cycle animation.")

    def __init__(self, obj, parent=None):
        """ Call the constructor of the parent class """
        logger.info('%s initialization' % obj.name)
        GraspingRobot.__init__(self, obj, parent)

        # We define here the name of the human grasping hand:
        #self.hand_name = 'Hand_Grab.R'

        armatures = blenderapi.get_armatures(self.bge_object)
        if len(armatures) == 0:
            logger.error("The human <%s> has not armature. Something is wrong!" % obj.name)
            return
        if len(armatures) > 1:
            logger.warning("The human <%s> has more than one armature. Using the first one" % obj.name)

        self.armature = armatures[0]

        logger.info('Component initialized')

        self.warnedAboutControlType = False

    def apply_speed(self, kind, linear_speed, angular_speed):
        """
        Apply speed parameter to the human.

        This overloaded version of Robot.apply_speed manage the walk/rest
        animations of the human avatar.

        :param string kind: not used. Forced to 'Position' control for now.
        :param list linear_speed: the list of linear speed to apply, for
        each axis, in m/s.
        :param list angular_speed: the list of angular speed to apply,
        for each axis, in rad/s.
        """

        # TODO: adjust this value depending on linear_speed to avoid 'slipping'
        speed_factor = 1.0

        # start/end frames of walk cycle in human_rig.blend
        # TODO: get that automatically from the timeline?
        WALK_START_FRAME = 9
        WALK_END_FRAME = 32

        if self._animations:
            if linear_speed[0] != 0 or angular_speed[2] != 0:
                self.armature.playAction("walk", 
                                        WALK_START_FRAME, WALK_END_FRAME, 
                                        speed=speed_factor)
            else:
                self.armature.playAction("walk", 
                                        WALK_START_FRAME, WALK_START_FRAME)


        if kind != 'Position':
            if not self.warnedAboutControlType:
                logger.error("Only the control type 'Position' is currently supported "
                            "by the human avatar! You need to configure accordingly"
                            " your motion actuator (for example, "
                            "\"motion.properties(ControlType='Position')\")")
                self.warnedAboutControlType = True
        else:
            GraspingRobot.apply_speed(self, 'Position', linear_speed, angular_speed)

    def default_action(self):
        """ Main function of this component. """
        pass
