import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.actuator
from morse.core.services import service
from math import pi
from morse.core import mathutils
from morse.helpers.components import add_data

class MocapControl(morse.core.actuator.Actuator):
    """ 
    Read the positions of the different joints at:
    head, neck, shoulders, torso, elbows, hands, hips, knees and feet
    And apply those positions to the control points (Empty's) of the armature
    """

    _name = "Mocap controller"
    _short_desc = "Controller for the motion of the human avatar, using the \
                   ASUS Xtion"

    add_data('head_position', [0.0, 0.0, 0.0], "vec3<float>",
             "Head position")
    add_data('neck_position', [0.0, 0.0, 0.0], "vec3<float>",
             "Neck position")
    add_data('left_hand_position', [0.0, 0.0, 0.0], "vec3<float>",
             "Left Hand position")
    add_data('right_hand_position', [0.0, 0.0, 0.0], "vec3<float>",
             "Right Hand position")
    add_data('left_elbow_position', [0.0, 0.0, 0.0], "vec3<float>",
             "Left elbow position")
    add_data('right_elbow_position', [0.0, 0.0, 0.0], "vec3<float>",
             "Right elbow position")
    add_data('left_shoulder_position', [0.0, 0.0, 0.0], "vec3<float>",
             "Left shoulder position")
    add_data('right_shoulder_position', [0.0, 0.0, 0.0], "vec3<float>",
             "Right shoulder position")
    add_data('left_hip_position', [0.0, 0.0, 0.0], "vec3<float>",
             "Left hip position")
    add_data('right_hip_position', [0.0, 0.0, 0.0], "vec3<float>",
             "Right hip position")
    add_data('left_foot_position', [0.0, 0.0, 0.0], "vec3<float>",
             "Left foot position")
    add_data('right_foot_position', [0.0, 0.0, 0.0], "vec3<float>",
             "Right foot position")
    add_data('torso_position', [0.0, 0.0, 0.0], "vec3<float>",
            "torso position")
    add_data('left_knee_position', [0.0, 0.0, 0.0], "vec3<float>",
            "left knee position")
    add_data('right_knee_position', [0.0, 0.0, 0.0], "vec3<float>",
            "right knee position")

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        parent = self.robot_parent

        self.human_position = parent.bge_object.worldPosition

        # Find the IK target empties
        for obj in parent.bge_object.childrenRecursive:
            #if obj.name == "Head_Empty":# in obj.name:
            if "Head_Empty" in obj.name:
                self._head_empty = obj
            #if obj.name == "Neck_Empty":# in obj.name:
            if "Neck_Empty" in obj.name:
                self._neck_empty = obj
            #if obj.name == "Torso_Empty":# in obj.name:
            if "Torso_Empty" in obj.name:
                self._torso_empty = obj
            #if obj.name == "Hand_Empty.L":# in obj.name:
            if "Hand_Empty.L" in obj.name:
                self._hand_empty_l = obj
            #if obj.name == "Hand_Empty.R":# in obj.name:
            if "Hand_Empty.R" in obj.name:
                self._hand_empty_r = obj
            #if obj.name == "Elbow_Empty.L":# in obj.name:
            if "Elbow_Empty.L" in obj.name:
                self._elbow_empty_l = obj
            #if obj.name == "Elbow_Empty.R":# in obj.name:
            if "Elbow_Empty.R" in obj.name:
                self._elbow_empty_r = obj
            #if obj.name == "Shoulder_Empty.L":# in obj.name:
            if "Shoulder_Empty.L" in obj.name:
                self._shoulder_empty_l = obj
            #if obj.name == "Shoulder_Empty.R":# in obj.name:
            if "Shoulder_Empty.R" in obj.name:
                self._shoulder_empty_r = obj
            #if obj.name == "Hip_Empty.L":# in obj.name:
            if "Hip_Empty.L" in obj.name:
                self._hip_empty_l = obj
            #if obj.name == "Hip_Empty.R":# in obj.name:
            if "Hip_Empty.R" in obj.name:
                self._hip_empty_r = obj
            #if obj.name == "Knee_Empty.L":# in obj.name:
            if "Knee_Empty.L" in obj.name:
                self._knee_empty_l = obj
            #if obj.name == "Knee_Empty.R":# in obj.name:
            if "Knee_Empty.R" in obj.name:
                self._knee_empty_r = obj
            #if obj.name == "Foot_Empty.L":# in obj.name:
            if "Foot_Empty.L" in obj.name:
                self._foot_empty_l = obj
            #if obj.name == "Foot_Empty.R":# in obj.name:
            if "Foot_Empty.R" in obj.name:
                self._foot_empty_r = obj

        logger.info('Component initialized')


    def default_action(self):
        """ Apply the positions read to the IK targets of the joints """

        # Compute a rotation angle for the whole body, based on the
        # angle of the shoulders
        world_x_vector = mathutils.Vector([1, 0, 0])
        shoulders_vector = mathutils.Vector([
            (self._shoulder_empty_l.worldPosition[0] - \
             self._shoulder_empty_r.worldPosition[0]),
            (self._shoulder_empty_l.worldPosition[1] - \
             self._shoulder_empty_r.worldPosition[1]),
            (self._shoulder_empty_l.worldPosition[2] - \
             self._shoulder_empty_r.worldPosition[2])  ])
        logger.debug ("Shoulder Vector: [%.4f, %.4f, %.4f]" % \
            (shoulders_vector[0], shoulders_vector[1], shoulders_vector[2]))

        try:
            # Measure the angle with respect to the X axis (in front of the man)
            body_rotation = shoulders_vector.angle(world_x_vector)
            # Correct the angle
            body_rotation -= pi/2
        except ValueError:
            # There will be an error if there is no user being tracked
            # by the Kinect
            body_rotation = 0.0
        logger.debug ("Angle with Y vector = %.4f" % body_rotation)

        # Apply the rotation to the toroso. The rest of the body should follow
        self._torso_empty.worldOrientation = [0.0, 0.0, body_rotation]

        # Put the ik targets in the correct positions
        self._set_object_position(self._head_empty.worldPosition,
                                  self.local_data['head_position'])
        self._set_object_position(self._neck_empty.worldPosition,
                                  self.local_data['neck_position'])
        self._set_object_position(self._torso_empty.worldPosition,
                                  self.local_data['torso_position'])
        self._set_object_position(self._hand_empty_l.worldPosition,
                                  self.local_data['left_hand_position'])
        self._set_object_position(self._hand_empty_r.worldPosition,
                                  self.local_data['right_hand_position'])
        self._set_object_position(self._elbow_empty_l.worldPosition,
                                  self.local_data['left_elbow_position'])
        self._set_object_position(self._elbow_empty_r.worldPosition,
                                  self.local_data['right_elbow_position'])
        self._set_object_position(self._shoulder_empty_l.worldPosition,
                                  self.local_data['left_shoulder_position'])
        self._set_object_position(self._shoulder_empty_r.worldPosition,
                                  self.local_data['right_shoulder_position'])
        self._set_object_position(self._hip_empty_l.worldPosition,
                                  self.local_data['left_hip_position'])
        self._set_object_position(self._hip_empty_r.worldPosition,
                                  self.local_data['right_hip_position'])
        self._set_object_position(self._knee_empty_l.worldPosition,
                                  self.local_data['left_knee_position'])
        self._set_object_position(self._knee_empty_r.worldPosition,
                                  self.local_data['right_knee_position'])
        self._set_object_position(self._foot_empty_l.worldPosition,
                                  self.local_data['left_foot_position'])
        self._set_object_position(self._foot_empty_r.worldPosition,
                                  self.local_data['right_foot_position'])

        #self._print_position("LEFT EMPTY", self._hand_empty_l.worldPosition)
        #self._print_position("RIGHT EMPTY", self._hand_empty_r.worldPosition)
        #self._print_position("HEAD", self._head_empty.worldPosition)
        #self._print_position("SHOULDER.L", \
        #                      self._shoulder_empty_l.worldPosition)
        #self._print_position("SHOULDER.R", \
        #                     self._shoulder_empty_r.worldPosition)

    def _set_object_position(self, joint, position):
        """ Add the position of the control point to the position of the
        Human robot
        """
        for i in range(3):
            joint[i] = self.human_position[i] + position[i]


    def _print_position(self, position_name, data):
        """ Format the data from the positions,
        in a way that the logger will like """
        text = "%s POS = [%.4f, %.4f, %.4f]" % \
               (position_name, data[0], data[1], data[2])
        logger.debug (text)
