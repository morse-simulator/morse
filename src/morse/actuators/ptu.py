import logging; logger = logging.getLogger("morse." + __name__)
import GameLogic
import math
import morse.core.actuator
from morse.core import status
from morse.core.services import service
from morse.core.services import async_service
import morse.helpers.math as morse_math

class PTUActuatorClass(morse.core.actuator.MorseActuatorClass):
    """ Controller for pant tilt unit (platine)

    Reads 2 angles (in radians) and applies them to the object and its children
    """

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        # Get the references to the childen object and
        #  store a transformation3d structure for their position
        for child in self.blender_obj.childrenRecursive:
            if 'PanBase' in child.name:
                self._pan_base = child
                self._pan_position_3d = morse.helpers.transformation.Transformation3d(child)
            elif 'TiltBase' in child.name:
                self._tilt_base = child
                self._tilt_position_3d = morse.helpers.transformation.Transformation3d(child)

        # Any other objects children of the PTU are assumed
        #  to be mounted on top of it
        for child in self.blender_obj.children:
            if not 'PanBase' in child.name:
                child.setParent(self._tilt_base)

        # Check the bases were found, or exit with a message
        try:
            logger.info("Using pan base: '%s'" % self._pan_base.name)
            logger.info("Using tilt base: '%s'" % self._tilt_base.name)
        except AttributeError as detail:
            logger.error("PTU is missing the pan and tilt bases. Module will not work!")
            return

        self._speed = self.blender_obj['Speed']
        # Define the tolerance to the desired angle
        try:
            self._tolerance = self.blender_obj['Tolerance']
        except KeyError as detail:
            self._tolerance = math.radians(0.3)
            logger.warn("Component '%s' does not have a Logic Property\n\t%s\n\tUsing default value = %.4f radians" % (obj.name, detail, self._tolerance))
        
        try:
            self._is_manual_mode = self.blender_obj['Manual']
        except KeyError:
            self._is_manual_mode = False 

        # Variables to store current angles
        self._current_pan = 0.0
        self._current_tilt = 0.0

        self.local_data['pan'] = 0.0
        self.local_data['tilt'] = 0.0
        
        logger.info('Component initialized')

    @async_service
    def set_pan_tilt(self, pan, tilt):
        """ """
        self.local_data['pan'] = pan
        self.local_data['tilt'] = tilt

    @service
    def get_pan_tilt(self):
       """
           Return the current angles for the pan and tilt segments.
       """
       return self._current_pan, self._current_tilt

    def default_action(self):
        """ Apply rotation to the platine unit """
        # Reset movement variables
        rx, ry, rz = 0.0, 0.0, 0.0

        if self._is_manual_mode:
            return

        # Update the postition of the base platforms
        try:
            self._pan_position_3d.update(self._pan_base)
            self._tilt_position_3d.update(self._tilt_base)
        except AttributeError as detail:
            logger.error("Platine is missing the pan and tilt bases. Platine does not work!")
            return

        # Tick rate is the real measure of time in Blender.
        # By default it is set to 60, regardles of the FPS
        # If logic tick rate is 60, then: 1 second = 60 ticks
        ticks = GameLogic.getLogicTicRate()

        try:
            normal_speed = self._speed / ticks
        # For the moment ignoring the division by zero
        # It happens apparently when the simulation starts
        except ZeroDivisionError:
            pass

        current_pan = self._pan_position_3d.yaw
        current_tilt = self._tilt_position_3d.pitch

        logger.debug("Platine: pan=%.4f, tilt=%.4f" % (current_pan, current_tilt))

        # Get the angles in a range of -PI, PI
        target_pan = morse_math.normalise_angle(self.local_data['pan'])
        target_tilt = morse_math.normalise_angle(self.local_data['tilt'])
        logger.debug("Targets: pan=%.4f, tilt=%.4f" % (target_pan, target_tilt))

        # Get the current rotation of the parent robot
        parent_pan = self.robot_parent.position_3d.euler.z
        parent_tilt = self.robot_parent.position_3d.euler.y
        logger.debug("Parent: pan=%.4f, tilt=%.4f" % (parent_pan, parent_tilt))

        # Compute the rotation relative to the parent robot
        relative_pan = current_pan - parent_pan
        correct_pan = morse_math.normalise_angle(relative_pan)
        relative_tilt = current_tilt - parent_tilt
        correct_tilt = morse_math.normalise_angle(relative_tilt)

        # Store the variables to acces as a service:
        self._current_pan = correct_pan
        self._current_tilt = correct_tilt

        if (abs(target_pan - correct_pan) < self._tolerance and \
            abs(target_tilt - correct_tilt) < self._tolerance ):
            self.completed((status.SUCCESS))

        # Determine the direction of the rotation, if any
        ry = morse_math.rotation_direction(correct_tilt, target_tilt, self._tolerance, normal_speed)
        rz = morse_math.rotation_direction(correct_pan, target_pan, self._tolerance, normal_speed)

        # Give the rotation instructions directly to the parent
        # The second parameter specifies a "local" movement
        self._pan_base.applyRotation([0.0, 0.0, rz], True)
        self._tilt_base.applyRotation([0.0, ry, 0.0], True)
