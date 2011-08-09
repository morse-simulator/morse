import logging; logger = logging.getLogger("morse." + __name__)
import math
import morse.core.sensor
import mathutils
import morse.helpers.math as morse_math

class KukaPostureClass(morse.core.sensor.MorseSensorClass):
    """ KUKA posture sensor """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self.local_data['x'] = 0.0
        self.local_data['y'] = 0.0
        self.local_data['z'] = 0.0
        self.local_data['yaw'] = 0.0
        self.local_data['pitch'] = 0.0
        self.local_data['roll'] = 0.0
        self.local_data['seg0'] = 0.0
        self.local_data['seg1'] = 0.0
        self.local_data['seg2'] = 0.0
        self.local_data['seg3'] = 0.0
        self.local_data['seg4'] = 0.0
        self.local_data['seg5'] = 0.0
        self.local_data['seg6'] = 0.0
        logger.info('Component initialized')

        # The axis along which the different segments rotate
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

        # Gather information about all segments of the kuka-arm
        self._segments = []

        kuka_obj = 0
        # Check if robot parent has a child named "kuka_base"
        for child in self.robot_parent.blender_obj.children:
            if str(child) == self.blender_obj['KUKAname']:
                kuka_obj = child

        #if kuka_obj != 0:
            logger.debug("Found kuka_arm")
        #else:
            logger.debug("WARNING: Kuka arm not found!")

        segment = kuka_obj.children[0]
        self._angles = []
        
        # Gather all the children of the object which are the segments of the kuka-arm
        for i in range(len(self._dofs)):
            self._segments.append(segment)
                   
            # Extract the angles
            rot_matrix = segment.localOrientation
            segment_matrix = mathutils.Matrix(rot_matrix[0], rot_matrix[1], rot_matrix[2])
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
                        
        # Store the data acquired by this sensor that could be sent
        #  via a middleware.
        self.local_data['x'] = float(x)
        self.local_data['y'] = float(y)
        self.local_data['z'] = float(z)    
        self.local_data['yaw'] = float(yaw)
        self.local_data['pitch'] = float(pitch)
        self.local_data['roll'] = float(roll)
        self.local_data['seg0'] = self._angles[0]
        self.local_data['seg1'] = self._angles[1]
        self.local_data['seg2'] = self._angles[2]
        self.local_data['seg3'] = self._angles[3]
        self.local_data['seg4'] = self._angles[4]
        self.local_data['seg5'] = self._angles[5]
        self.local_data['seg6'] = self._angles[6]

