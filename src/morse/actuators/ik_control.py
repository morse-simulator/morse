import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.actuator
from morse.core.services import service

class IKControlClass(morse.core.actuator.MorseActuatorClass):
    """ Controller for the hands of the human robot, using Inverse Kinematics

    Should read the positions of the hand endpoints and pass them
    to the human armature. IK should take care of the rest of the
    arm position.
    The data can come from a motion capture system or the Kinect
    """

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        parent = self.robot_parent
        # Find the IK target empties
        for obj in parent.blender_obj.children:
            if "IK_Target_Empty.L" in obj.name:
                self._left_ik_target = obj
            if "IK_Target_Empty.R" in obj.name:
                self._right_ik_target = obj
            # Also get the target for the head
            if "IK_Target_Empty.Head" in obj.name:
                self._head_ik_target = obj

        self.local_data['left_hand_position'] = [0.3, 0.3, 0.9]
        self.local_data['right_hand_position'] = [0.3, -0.3, 0.9]
        #self.local_data['left_hand_rotation'] = [0.0, 0.0, 0.0]
        #self.local_data['right_hand_rotation'] = [0.0, 0.0, 0.0]
        #self.local_data['head_position'] = [0.5, 0.0, 1.6]

        logger.info('Component initialized')
        logger.setLevel(logging.DEBUG)


    def default_action(self):
        """ Apply the positions read to the IK targets of the hands """
        self._left_ik_target.localPosition = self.local_data['left_hand_position']
        self._right_ik_target.localPosition = self.local_data['right_hand_position']
        self._head_ik_target.localPosition = self.local_data['head_position']


        #self._print_position("LEFT EMPTY", self._left_ik_target.localPosition)
        #self._print_position("RIGHT EMPTY", self._right_ik_target.localPosition)
        #self._print_position("HEAD", self._head_ik_target.localPosition)


    def _print_position(self, position_name, data):
        """ Format the data from the positions,
        in a way that the logger will like """
        text = "%s POS = [%.4f, %.4f, %.4f]" % (position_name, data[0], data[1], data[2])
        logger.debug (text)
