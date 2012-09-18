import logging; logger = logging.getLogger("morse." + __name__)
import math
import morse.core.actuator
from morse.core import blenderapi
from morse.core import mathutils
from morse.helpers.morse_math import normalise_angle, rotation_direction
from morse.core.services import service
from morse.helpers.components import add_data, add_property

class PA10ActuatorClass(morse.core.actuator.MorseActuatorClass):
    """ Motion controller using linear and angular speeds

    This component will read an array of 6 floats, and apply them as
    rotation for the parts of the PA-10 arm.
    """

    _name = "PA-10"
    _short_desc = "PA-10 6-DOF robotic arm"

    add_property('_speed', 1.0, "Speed", "float", 'speed of each joint, in rad/s')
    add_property('_tolerance', math.radians(5), "Tolerance", "float", 'tolerance on the position, in radians')

    add_data('seg0', 0.0, "float", "first joint (base)")
    add_data('seg1', 0.0, "float", "second joint")
    add_data('seg2', 0.0, "float", "third joint")
    add_data('seg3', 0.0, "float", "fourth joint")
    add_data('seg4', 0.0, "float", "fifth joint")
    add_data('seg5', 0.0, "float", "sixth joint (wrist)")

    def __init__(self, obj, parent=None):
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        # The axis along which the different segments rotate
        # Considering the rotation of the arm as installed in Jido
        self._dofs = ['z', 'y', 'y', 'z', 'y', 'z']

        self._segments = []
        segment = self.blender_obj.children[0]
        for i in range(6):
            self._segments.append(segment)
            try:
                segment = segment.children[0]
            except IndexError as error:
                break
        logger.info ("Arm segment list: ", self._segments)

        # Get the references to the segment at the tip of the arm
        for child in self.blender_obj.childrenRecursive:
            if 'PA10-6' in child.name:
                self._arm_tip = child
                break

        # Any other objects children of the Kuka arm are assumed
        #  to be mounted on the tip of the arm
        for child in self.blender_obj.children:
            if not 'PA10' in child.name:
                child.setParent(self._arm_tip)

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
            contr = blenderapi.controller()
            self._sound = contr.actuators['Sound']
            contr.activate(self._sound)
            self._sound.stopSound()

        # Reset movement variables
        rx, ry, rz = 0.0, 0.0, 0.0

        # Scale the speeds to the time used by Blender
        try:
            rotation = _speed / self.frequency
        # For the moment ignoring the division by zero
        # It happens apparently when the simulation starts
        except ZeroDivisionError:
            pass

        self._moving = False

        for i in range(6):
            key = ('seg%d' % i)
            target_angle = normalise_angle(self.local_data[key])

            # Get the next segment
            segment = self._segments[i]

            # Extract the angles
            rot_matrix = segment.localOrientation
            segment_matrix = mathutils.Matrix((rot_matrix[0], rot_matrix[1], rot_matrix[2]))
            segment_euler = segment_matrix.to_euler()

            # Use the corresponding direction for each rotation
            if self._dofs[i] == 'y':
                ry = rotation_direction(segment_euler[1], target_angle, _tolerance, rotation)
                #logger.debug("PARAMETERS Y: %.4f, %.4f, %.4f, %.4f = %.4f" % (segment_euler[1], target_angle, _tolerance, rotation, ry))

            elif self._dofs[i] == 'z':
                rz = rotation_direction(segment_euler[2], target_angle, _tolerance, rotation)
                #logger.debug("PARAMETERS Z: %.4f, %.4f, %.4f, %.4f = %.4f" % (segment_euler[2], target_angle, _tolerance, rotation, rz))

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


    @service
    def set_rotation_array(self, seg0=0, seg1=0, seg2=0, seg3=0, seg4=0, seg5=0):
        """
        MORSE service to set the rotation for each of the arm joints.
        It receives an array containing the angle to give to each of
        the robot articulations. The array contains only one angle for
        each joint.

        :param seg0: 1st joint angle (base)
        :param seg1: 2nd joint angle
        :param seg2: 3rd joint angle
        :param seg3: 4th joint angle
        :param seg4: 5th joint angle
        :param seg5: 6th joint angle (wrist)
        """
        self.local_data['seg0'] = seg0
        self.local_data['seg1'] = seg1
        self.local_data['seg2'] = seg2
        self.local_data['seg3'] = seg3
        self.local_data['seg4'] = seg4
        self.local_data['seg5'] = seg5
        return None
