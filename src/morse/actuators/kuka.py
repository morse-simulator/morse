import logging; logger = logging.getLogger("morse." + __name__)
import GameLogic
import math
import sys
import mathutils
import morse.core.actuator
import morse.helpers.math as morse_math

class KukaActuatorClass(morse.core.actuator.MorseActuatorClass):
    """ Arm control for the Kuka arm using angles

    This component will read an array of 7 floats, and apply them as
    rotation for the parts of the Kuka LWR arm.
    """

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self._speed = self.blender_obj['Speed']
        # Define a tolerance for the angles as inputs
        self._tolerance = math.radians(0.5)

        self.local_data['seg0'] = 0.0
        self.local_data['seg1'] = 0.0
        self.local_data['seg2'] = 0.0
        self.local_data['seg3'] = 0.0
        self.local_data['seg4'] = 0.0
        self.local_data['seg5'] = 0.0
        self.local_data['seg6'] = 0.0

        # The axis along which the different segments rotate
        # Considering the rotation of the arm as installed in Jido
        self._dofs = ['z', '-y', 'z', 'y', 'z', '-y', 'z']

        self._segments = []
        segment = self.blender_obj.children[0]
        # Gather all the children of the object
        while True:
            self._segments.append(segment)
            try:
                segment = segment.children[0]
            # Exit when there are no more children
            except IndexError as detail:
                break

        logger.info('Component initialized')



    def default_action(self):
        """ Apply rotation to the arm segments """

        # Reset movement variables
        rx, ry, rz = 0.0, 0.0, 0.0

        # Tick rate is the real measure of time in Blender.
        # By default it is set to 60, regardles of the FPS
        # If logic tick rate is 60, then: 1 second = 60 ticks
        ticks = GameLogic.getLogicTicRate()
        # Scale the speeds to the time used by Blender
        try:
            rotation = self._speed / ticks
        # For the moment ignoring the division by zero
        # It happens apparently when the simulation starts
        except ZeroDivisionError:
            pass

        # Use the length of _dofs, since it won't change.
        # The length of _segments will change if more objects are attached
        #  at the end of the arm, as in the case of a hand
        for i in range(len(self._dofs)):
            key = ('seg%d' % i)
            target_angle = morse_math.normalise_angle(self.local_data[key])

            # Get the next segment
            segment = self._segments[i]

            # Extract the angles
            rot_matrix = segment.localOrientation
            
            # temporarily support old Blender2.56-code using Python3.1 instead of Python3.2
            if sys.version_info.minor == 1:
                segment_matrix = mathutils.Matrix(rot_matrix[0], rot_matrix[1], rot_matrix[2])
            else:
                segment_matrix = mathutils.Matrix((rot_matrix[0], rot_matrix[1], rot_matrix[2])) 

            segment_euler = segment_matrix.to_euler()

            # Use the corresponding direction for each rotation
            if self._dofs[i] == 'y':
                ry = morse_math.rotation_direction(segment_euler[1], target_angle, self._tolerance, rotation)
            elif self._dofs[i] == '-y':
                ry = morse_math.rotation_direction(segment_euler[1], -target_angle, self._tolerance, rotation)
            elif self._dofs[i] == 'z':
                rz = morse_math.rotation_direction(segment_euler[2], target_angle, self._tolerance, rotation)
                logger.debug("PARAMETERS: %.4f, %.4f, %.4f, %.4f = %.4f" % (segment_euler[2], target_angle, self._tolerance, rotation, rz))

            # Give the movement instructions directly to the parent
            # The second parameter specifies a "local" movement
            segment.applyRotation([rx, ry, rz], True)

            # Reset the rotations for the next segment
            ry = rz = 0
