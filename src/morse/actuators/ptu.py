import logging; logger = logging.getLogger("morse." + __name__)
import math
import morse.core.actuator
from morse.core import blenderapi
from morse.core.actuator import MorseActuatorClass
from morse.core import status
from morse.core.services import service
from morse.core.services import async_service
from morse.core.services import interruptible
import morse.helpers.math as morse_math

class PTUActuatorClass(morse.core.actuator.MorseActuatorClass):
    """ Generic controller for pan-tilt units

    Reads 2 angles (in radians) and applies them to the object and its children.
    """

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        # Get the references (based on their name) to the childen object and
        # store a transformation3d structure for their position
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
            logger.error("PTU is missing the pan and/or tilt bases. Module will not work!")
            return

        # Initialises a couple of properties. They can be changed by Builder scripts
        self.add_property('_speed', 1.0, 'Speed')
        self.add_property('_tolerance', math.radians(0.3), 'Tolerance')
        self.add_property('_is_manual_mode', False, 'Manual')

        # Variables to store current angles
        self._current_pan = 0.0
        self._current_tilt = 0.0

        self.local_data['pan'] = 0.0
        self.local_data['tilt'] = 0.0

        logger.info('Component initialized')


    @interruptible
    @async_service
    def set_pan_tilt(self, pan, tilt):
        """ Asynchronous, interruptible service that moves the PTU to a given
        target position.
        """

        logger.debug("Service 'set_pan_tilt' setting angles to %.4f, %.4f" % (pan, tilt))
        self.local_data['pan'] = pan
        self.local_data['tilt'] = tilt

    @service
    def get_pan_tilt(self):
        """ Returns the current angles for the pan and tilt segments. """
        return self._current_pan, self._current_tilt

    @interruptible
    @async_service
    def look_at_point(self, x, y, z):
        """ Interruptible, asynchronous service to make the camera look towards
        a given point.

        Coordinates must be given in the world reference.
        """
        self._aim_camera_at_point(x, y, z)

    @interruptible
    @async_service
    def look_at_object(self, obj_name):
        """ Look in the direction of the given object.
        
        :param obj_name: the (Blender) name of an object present in the scene
        """
        scene = blenderapi.scene()
        try:
            obj = scene.objects[obj_name]
        except KeyError as detail:
            logger.error("Object '%s' not found in scene. Can not look at it" % obj_name)
            return False

        logger.debug ("Found object '%s'" % obj)
        self._aim_camera_at_point(obj.worldPosition[0], obj.worldPosition[1], obj.worldPosition[2])


    def _aim_camera_at_point(self, x, y, z):
        """ Turn the unit to face a given point in space

        Receive the coordinates of the point to look at,
        given in world coordinates.

        Fill local_data with the corresponding pan and tilt to aim in that
        direction.

        Use the formulas at
        http://en.wikipedia.org/wiki/Spherical_coordinate_system#Cartesian_coordinates
        """
        goalPos = [0, 0, 0]

        # Get the postitions with respect to the PTU
        goalPos[0] = x - self.position_3d.x
        goalPos[1] = y - self.position_3d.y
        goalPos[2] = z - self.position_3d.z

        logger.debug("target  = [%.4f, %.4f, %.4f]" % (x, y, z))
        logger.debug("goalPos = [%.4f, %.4f, %.4f]" % (goalPos[0], goalPos[1], goalPos[2]))


        distance = math.sqrt(goalPos[0] ** 2 + goalPos[1] ** 2 + goalPos[2] ** 2)
        theta = math.asin(goalPos[2] / distance)
        phi = math.atan2(goalPos[1], goalPos[0])

        logger.debug("Theta = %.4f | Phi = %.4f" % (theta, phi))

        # Get the current rotation of the parent robot
        parent_pan = self.robot_parent.position_3d.euler.z
        parent_tilt = self.robot_parent.position_3d.euler.y

        # Adjust the rotation with respect to the parent
        self.local_data['pan'] = phi - parent_pan
        self.local_data['tilt'] = -theta + parent_tilt


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
            logger.error("The PTU is missing the pan and/or tilt bases. Discarding action.")
            return

        try:
            normal_speed = self._speed / self.frequency
        # For the moment ignoring the division by zero
        # It happens apparently when the simulation starts
        except ZeroDivisionError:
            pass

        current_pan = self._pan_position_3d.yaw
        current_tilt = self._tilt_position_3d.pitch

        logger.debug("PTU: pan=%.4f, tilt=%.4f" % (current_pan, current_tilt))

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
            abs(target_tilt - correct_tilt) < self._tolerance):
            self.completed((status.SUCCESS))

        # Determine the direction of the rotation, if any
        ry = morse_math.rotation_direction(correct_tilt, target_tilt, self._tolerance, normal_speed)
        rz = morse_math.rotation_direction(correct_pan, target_pan, self._tolerance, normal_speed)

        # Give the rotation instructions directly to the parent
        # The second parameter specifies a "local" movement
        self._pan_base.applyRotation([0.0, 0.0, rz], True)
        self._tilt_base.applyRotation([0.0, ry, 0.0], True)
