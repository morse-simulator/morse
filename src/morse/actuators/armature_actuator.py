import logging; logger = logging.getLogger("morse." + __name__)
import math
import morse.core.actuator
from morse.core.services import service
from morse.core.exceptions import MorseRPCInvokationError


class ArmatureActuatorClass(morse.core.actuator.MorseActuatorClass):
    """
    Class to represent an actuator to actuate on blender armatures in MORSE.

    Sub class of MorseActuatorClass.
    This class has many MORSE Services that you can access via sockets/telnet.
    """


    def __init__(self, obj, parent=None):
        """
        Constructor method.
        Receives the reference to the Blender object.
        """     
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(ArmatureActuatorClass,self).__init__(obj, parent)
        
        # Initialize the values in local_data for each segment
        armature = self.blender_obj
        for channel in armature.channels:
            self.local_data[channel.name] = 0.0

        # The axis along which the different segments rotate
        # Considering the constraints defined for the armature
        #  in the Blender file
        self._dofs = self.get_dofs()

        logger.info('Component initialized')
        

    @service
    def get_channels(self):
        """
        MORSE Service that returns a list of the channels (bones) of the armature.
        """
        armature = self.blender_obj
        channels = []
        # add the name of each channel to the list
        for channel in armature.channels:
            channels.append(channel.name)
        
        return channels

    @service
    def get_location(self, channel_name):
        """
        MORSE Service that returns the location corresponding to
        the given channel 'channel_name' on the armature.
        """
        armature = self.blender_obj
        channel = armature.channels[str(channel_name)]
        # get the rotation in xyz
        rotation = channel.location

        return rotation

    def set_location(self, channel_name, location):
        """
        Method that sets the location of the given channel to location.
        We do NOT take into account the IK limits here.
        """
        armature = self.blender_obj
        channel = armature.channels[str(channel_name)]
        channel.location = location
        armature.update()
        
    @service
    def get_rotations(self):
        """
        MORSE Service that returns a dict with keys the channel names of
        the armature and values the rotation xyz values.
        """
        armature = self.blender_obj
        rotations = {}
        # get the rotation of each channel
        for channel in armature.channels:
            rotations[channel.name] = channel.joint_rotation.to_tuple()

        return rotations

    @service
    def get_rotation(self, channel_name):
        """
        MORSE Service that returns the rotation angles corresponding to
        the given channel 'channel_name' on the armature.
        """
        armature = self.blender_obj
        try : 
            channel = armature.channels[str(channel_name)]
            # get the rotation in xyz
            rotation = channel.joint_rotation.to_tuple()

            return rotation
        except KeyError:
            msg = str(channel_name) + " is not a valid channel name"
            raise MorseRPCInvokationError(msg)

    def find_dof(self, channel):
        """
        Method that finds and returns the degree of freedom (dof) corresponding
        to the given channel.
        The dof has to be a blender_ik_setting.
        Returns a list [x,y,z] with the corresponding dofs as a boolean.
        """
        return [channel.ik_dof_x, 
                channel.ik_dof_y,
                channel.ik_dof_z]

    @service
    def get_dofs(self):
        """
        MORSE Service that returns a dictionary with keys the channels
        of the armature and as values a list [x,y,z] with a boolean corresponding
        indication if the axis (x,y,z) is a dof.
        """
        armature = self.blender_obj
        dofs = {}
        # find the dof of each channel
        for channel in armature.channels:
            dofs[channel.name] = self.find_dof(channel)

        return dofs

    def set_joint_rotation(self, armature, channel, rotation):
        """
        Method that sets the rotaion of the given channel to rotation.
        channel.joint_rotation takes in account the limits set via IK.
        """
        channel.joint_rotation = rotation
        armature.update()

    @service
    def set_rotation(self, channel_name, rotation):
        """
        MORSE Service to set the rotation angle of the given channel_name to the angles list (x,y,z).
        """
        armature = self.blender_obj
        try:
            channel = armature.channels[str(channel_name)]
            self.set_joint_rotation(armature, channel, rotation)
            return None
        except KeyError:
            msg = str(channel_name) + " is not a valid channel name"
            raise MorseRPCInvokationError(msg)

    @service
    def get_IK_minmax(self):
        """
        MORSE Service to return a dictionary with keys the channel names of the armature
        and values the IK min and max limits in the following form:
        [[ik_min_x,ik_max_x], [ik_min_y,ik_max_y], [ik_min_z,ik_max_z]] (list of lists of floats)
        """
        armature = self.blender_obj
        minmax_dict = {}
        for channel in armature.channels:
            # find the min and max values for each channel
            lst = [[0,0],[0,0],[0,0]]
            lst[0][0] = channel.ik_min_x
            lst[0][1] = channel.ik_max_x
            lst[1][0] = channel.ik_min_y
            lst[1][1] = channel.ik_max_y
            lst[2][0] = channel.ik_min_z
            lst[2][1] = channel.ik_max_z
            minmax_dict[channel.name] = lst
        return minmax_dict
        
    @service
    def get_IK_limits(self):
        """
        MORSE Service to return a dict with keys the channel names of the armature
        and values the IK limits in the following form:
        [ik_limit_x,ik_limit_y,ik_limit_z] (list of booleans)
        """
        armature = self.blender_obj
        limits_dict = {}
        for channel in armature.channels:
            # find if the limits are enabled on the different axes
            limits_dict[channel.name] = [channel.ik_limit_x,
                                         channel.ik_limit_y,
                                         channel.ik_limit_z]
        return limits_dict

    @service
    def get_channel_lengths(self):
        """
        MORSE Service to return a dict with keys the channel names of the armature
        and values the channel's lenght.
        """
        armature = self.blender_obj
        lengths_dict = {}
        for channel in armature.channels:
            # find the length of the current channel
            #head = channel.pose_head
            #tail = channel.pose_tail
            diff = channel.pose_head - channel.pose_tail
            length = math.sqrt(diff[0]**2 + diff[1]**2 + diff[2]**2)
            lengths_dict[channel.name] = length
        return lengths_dict

    @service
    def get_robot_parent_name(self):
        """
        MORSE Service to return the blender name of the robot that is the parent of this armature.
        """
        return self.robot_parent.blender_obj.name
            

    def default_action(self):
        """
        Main function of this component.
        Is called every tick of the clock.
        """
        pass
