import logging; logger = logging.getLogger("morse." + __name__)
import math
import morse.actuators.armature_actuator
from morse.helpers.morse_math import normalise_angle
from morse.core.services import service
from morse.core.exceptions import MorseRPCInvokationError

class KukaActuatorClass(morse.actuators.armature_actuator.ArmatureActuatorClass):
    """
    Class to control the KUKA LWR arm using Blender armatures.
    Sub class of ArmatureActuatorClass, that considers the specific case
    of objects that can be mounted on the arm.
    This class has many MORSE Services that you can access via sockets/telnet.
    """

    def __init__(self, obj, parent=None):
        """
        Constructor method.
        Receives the reference to the Blender object.
        """     
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        # Get the references to the segment at the tip of the arm
        for child in self.blender_obj.childrenRecursive:
            if 'kuka_7' in child.name:
                self._arm_tip = child

        # Any other objects children of the Kuka arm are assumed
        #  to be mounted on the tip of the arm
        for child in self.blender_obj.children:
            if not 'kuka' in child.name:
                child.setParent(self._arm_tip)
        
        #self._tolerance = math.radians(0.5)

        logger.info('Component initialized')


    def default_action(self):
        """ Apply rotation angles to the segments of the arm """

        armature = self.blender_obj
        logger.debug("The armature is: '%s' (%s)" % (armature, type(armature)))

        for channel in armature.channels:
            segment_angle = channel.joint_rotation
            logger.debug("Channel '%s' st: [%.4f, %.4f, %.4f]" % (channel, segment_angle[0], segment_angle[1], segment_angle[2]))

            # Get the normalised angle for this segment
            target_angle = normalise_angle(self.local_data[channel.name])

            # Use the corresponding direction for each rotation
            if self._dofs[channel.name][1] == 1:
                segment_angle[1] = target_angle
            elif self._dofs[channel.name][2] == 1:
                segment_angle[2] = target_angle

            logger.debug("Channel '%s' fn: [%.4f, %.4f, %.4f]" % (channel, segment_angle[0], segment_angle[1], segment_angle[2]))
            channel.joint_rotation = segment_angle

            armature.update()


    @service
    def set_rotation(self, channel_name, rotation):
        """
        MORSE Service to set the rotation angle of the given channel_name
         to the angles list (x,y,z).
        Overrides the default method of the ArmatureActuatorClass, so that data
         is stored in the expected format of a list of angles for each joint.
        """
        try:
            for i in range(3):
                if self._dofs[channel_name][i] != 0:
                    self.local_data[channel_name] = rotation[i]
            return None
        except KeyError:
            msg = str(channel_name) + " is not a valid channel name "
            raise MorseRPCInvokationError(msg)

    @service
    def set_rotation_array(self, *rotation_array):
        """
        MORSE service to set the rotation for each of the arm joints.
        It receives an array containing the angle to give to each of
        the robot articulations. The array contains only one angle for
        each joint.
        """
        i = 0
        for channel in self.blender_obj.channels:
            try:
                self.local_data[channel.name] = rotation_array[i]
                i += 1
            # If there are no more arguments, set the rotation values to zero
            except IndexError:
                self.local_data[channel.name] = 0.0
        return None
