import logging; logger = logging.getLogger("morse." + __name__)
import GameLogic
import math
import morse.core.sensor
import mathutils
import morse.helpers.math as morse_math

class PTUPostureClass(morse.core.sensor.MorseSensorClass):
    """ KUKA posture sensor """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)
        
        # Check if robot parent has a child named "kuka_base"
        for child in self.robot_parent.blender_obj.children:
            if str(child) == self.blender_obj['PTUname']:
                self._ptu_obj = child
        
        # Get the references to the childen object and
        #  store a transformation3d structure for their position
        for child in self._ptu_obj.childrenRecursive:
            if 'PanBase' in child.name:
                self._pan_base = child
                self._pan_position_3d = morse.helpers.transformation.Transformation3d(child)
            elif 'TiltBase' in child.name:
                self._tilt_base = child
                self._tilt_position_3d = morse.helpers.transformation.Transformation3d(child)
        # Check the bases were found, or exit with a message
        try:
            logger.info("Using PTU: '%s'" % self._ptu_obj.name)
            logger.info("Using pan base: '%s'" % self._pan_base.name)
            logger.info("Using tilt base: '%s'" % self._tilt_base.name)
        except AttributeError as detail:
            logger.error("Platine is missing the pan and tilt bases. Module will not work!")
        self.local_data['pan'] = 0.0
        self.local_data['tilt'] = 0.0
        logger.info('Component initialized')

    def default_action(self):
        """ Apply rotation to the platine unit """
        # Reset movement variables
        rx, ry, rz = 0.0, 0.0, 0.0

        # Update the postition of the base platforms
        try:
            self._pan_position_3d.update(self._pan_base)
            self._tilt_position_3d.update(self._tilt_base)
        except AttributeError as detail:
            logger.error("Platine is missing the pan and tilt bases. Platine does not work!")
            return

        current_pan = self._pan_position_3d.yaw
        current_tilt = self._tilt_position_3d.pitch
        logger.debug("Platine: pan=%.4f, tilt=%.4f" % (current_pan, current_tilt))
        
        # Store the data acquired by this sensor that could be sent via a middleware.
        self.local_data['pan'] = float(current_pan)
        self.local_data['tilt'] = float(current_tilt)
