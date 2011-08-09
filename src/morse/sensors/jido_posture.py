import logging; logger = logging.getLogger("morse." + __name__)
import math
import morse.core.sensor
import mathutils
import sys
import morse.helpers.math as morse_math

class JidoPostureClass(morse.core.sensor.MorseSensorClass):
    """ Jido posture sensor. Currently working with PTU and KUKA arm """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        # Object position (maybe delete later)
        self.local_data['x'] = 0.0
        self.local_data['y'] = 0.0
        self.local_data['z'] = 0.0
        self.local_data['yaw'] = 0.0
        self.local_data['pitch'] = 0.0
        self.local_data['roll'] = 0.0

        # joints of kuka-arm
        self.local_data['seg0'] = 0.0
        self.local_data['seg1'] = 0.0
        self.local_data['seg2'] = 0.0
        self.local_data['seg3'] = 0.0
        self.local_data['seg4'] = 0.0
        self.local_data['seg5'] = 0.0
        self.local_data['seg6'] = 0.0

        # joints of PTU-unit
        self.local_data['pan'] = 0.0
        self.local_data['tilt'] = 0.0
        logger.info('Component initialized')
        
        ##################### PTU joints ##################

        # Check if robot parent has a child named "PTUname"
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

        ###################### KUKA joints ##################

        # Gather information about all segments of the kuka-arm
        self._segments = []

        self.kuka_obj = 0
        # Check if robot parent has a child named "kuka_base"
        for child in self.robot_parent.blender_obj.children:
            if str(child) == self.blender_obj['KUKAname']:
                self.kuka_obj = child

        try:
            logger.info("Using KUKA arm: '%s'" % self.kuka_obj.name)
        except AttributeError as detail:
            logger.error("Kuka arm is missing. Module will not work!")
       
        # The axis along which the different segments of the kuka armrotate
        # Considering the rotation of the arm as installed in Jido
        self._dofs = ['z', '-y', 'z', 'y', 'z', '-y', 'z']
       
    def default_action(self):
        """ Get the x, y, z, yaw, pitch and roll of the blender object. """
        x = self.position_3d.x
        y = self.position_3d.y
        z = self.position_3d.z
        yaw = self.position_3d.yaw
        pitch = self.position_3d.pitch
        roll = self.position_3d.roll

        ############################# PTU joints ############################## 

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

        ############################# KUKA joints ##############################

        segment = self.kuka_obj.children[0]
        self._angles = []
        
        # Gather all the children of the object which are the segments of the kuka-arm
        for i in range(len(self._dofs)):
            self._segments.append(segment)
                   
            # Extract the angles
            rot_matrix = segment.localOrientation
            if sys.version_info.minor == 1:
                segment_matrix = mathutils.Matrix(rot_matrix[0], rot_matrix[1], rot_matrix[2])
            else:
                segment_matrix = mathutils.Matrix((rot_matrix[0], rot_matrix[1], rot_matrix[2]))
           
            segment_euler = segment_matrix.to_euler()

            # Use the corresponding direction for each rotation
            if self._dofs[i] == 'y':
                self._angles.append(segment_euler[1])
            elif self._dofs[i] == '-y':
                self._angles.append(-segment_euler[1])
            elif self._dofs[i] == 'z':
                self._angles.append(segment_euler[2])

            try:
                segment = segment.children[0]
            # Exit when there are no more children
            except IndexError as detail:
                break
        ############################# Hand data over to middleware ##############################

        self.local_data['x'] = float(x)
        self.local_data['y'] = float(y)
        self.local_data['z'] = float(z)
        self.local_data['yaw'] = float(yaw)
        self.local_data['pitch'] = float(pitch)
        self.local_data['roll'] = float(roll)

        # KUKA arm
        self.local_data['seg0'] = self._angles[0]
        self.local_data['seg1'] = self._angles[1]
        self.local_data['seg2'] = self._angles[2]
        self.local_data['seg3'] = self._angles[3]
        self.local_data['seg4'] = self._angles[4]
        self.local_data['seg5'] = self._angles[5]
        self.local_data['seg6'] = self._angles[6]

        # PTU 
        self.local_data['pan'] = float(current_pan)
        self.local_data['tilt'] = float(current_tilt)
