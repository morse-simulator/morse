import logging; logger = logging.getLogger("morse." + __name__)
import bge
import math
import mathutils
import morse.core.actuator
import morse.helpers.math as morse_math

class PA10ActuatorClass(morse.core.actuator.MorseActuatorClass):
    """ Motion controller using linear and angular speeds

    This component will read an array of 6 floats, and apply them as
    rotation for the parts of the PA-10 arm.
    """

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self._speed = self.blender_obj['Speed']
        self._tolerance = math.radians(5)

        self.local_data['seg0'] = 0.0
        self.local_data['seg1'] = 0.0
        self.local_data['seg2'] = 0.0
        self.local_data['seg3'] = 0.0
        self.local_data['seg4'] = 0.0
        self.local_data['seg5'] = 0.0

        # The axis along which the different segments rotate
        # Considering the rotation of the arm as installed in Jido
        self._dofs = ['z', 'y', 'y', 'z', 'y', 'z']

        self._segments = []
        segment = self.blender_obj.children[0]
        for i in range(6):
            self._segments.append(segment)
            segment = segment.children[0]

        # Variable to store the reference to the Sound actuator
        self._sound = None

        self._moving = False

        logger.info('Component initialized')
        #logger.setLevel(logging.DEBUG)



    def default_action(self):
        """ Apply rotation to the arm segments """
        # Get the reference to the Sound actuator
        if self._sound == None:
            logger.debug ("ACTIVATING THE SOUND ACTUATOR")
            contr = bge.logic.getCurrentController()
            self._sound = contr.actuators['Sound']
            contr.activate(self._sound)
            self._sound.stopSound()

        # Reset movement variables
        rx, ry, rz = 0.0, 0.0, 0.0

        # Tick rate is the real measure of time in Blender.
        # By default it is set to 60, regardles of the FPS
        # If logic tick rate is 60, then: 1 second = 60 ticks
        ticks = bge.logic.getLogicTicRate()
        # Scale the speeds to the time used by Blender
        try:
            rotation = self._speed / ticks
        # For the moment ignoring the division by zero
        # It happens apparently when the simulation starts
        except ZeroDivisionError:
            pass

        self._moving = False

        for i in range(6):
            key = ('seg%d' % i)
            target_angle = morse_math.normalise_angle(self.local_data[key])

            # Get the next segment
            segment = self._segments[i]

            # Extract the angles
            rot_matrix = segment.localOrientation
            segment_matrix = mathutils.Matrix((rot_matrix[0], rot_matrix[1], rot_matrix[2]))
            segment_euler = segment_matrix.to_euler()

            # Use the corresponding direction for each rotation
            if self._dofs[i] == 'y':
                ry = morse_math.rotation_direction(segment_euler[1], target_angle, self._tolerance, rotation)
                #logger.debug("PARAMETERS Y: %.4f, %.4f, %.4f, %.4f = %.4f" % (segment_euler[1], target_angle, self._tolerance, rotation, ry))
            elif self._dofs[i] == 'z':
                rz = morse_math.rotation_direction(segment_euler[2], target_angle, self._tolerance, rotation)
                #logger.debug("PARAMETERS Z: %.4f, %.4f, %.4f, %.4f = %.4f" % (segment_euler[2], target_angle, self._tolerance, rotation, rz))

            logger.debug("ry = %.4f, rz = %.4f" % (ry, rz))

            # Give the movement instructions directly to the parent
            # The second parameter specifies a "local" movement
            segment.applyRotation([rx, ry, rz], True)

            if ry != 0.0 or rz != 0.0:
                self._moving = True

            # Reset the rotations for the next segment
            ry = rz = 0

        if self._moving:
            self._sound.startSound()
            logger.debug("STARTING SOUND")
        else:
            self._sound.stopSound()
            logger.debug("STOPPING SOUND")
