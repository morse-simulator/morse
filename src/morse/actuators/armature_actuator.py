# @Authors: Peter Roelants peter.roelants@gmail.com
# @Owner: KU Leuven - Dep. Mechanical Eng. - Robotics
# @File: armature_actuator.py
# @Description:  
# @License: 
# (C) 2010 Name, peter.roelants@gmail.com, Department of Mechanical
# Engineering, Katholieke Universiteit Leuven, Belgium.
#
# You may redistribute this software and/or modify it under either the terms of the GNU Lesser
# General Public License version 2.1
# (LGPLv2.1 <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>)
# or (at your discretion) of the Modified BSD License:
# Redistribution and use in source and binary forms, with or without modification, are permitted
# provided that the following conditions are met:
#  1. Redistributions of source code must retain the above copyright notice, this list of 
#     conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright notice, this list of 
#     conditions and the following disclaimer in the documentation and/or other materials
#     provided with the distribution.
#  3. The name of the author may not be used to endorse or promote products derived from
#     this software without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
# ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
# OF SUCH DAMAGE.
import logging; logger = logging.getLogger("morse." + __name__)
import GameLogic
import math
import morse.core.actuator
import morse.helpers.math as morse_math
from morse.core.services import service
from morse.core import status


class ArmatureActuatorClass(morse.core.actuator.MorseActuatorClass):
    """
    Class to represent an actuator to actuate on blender armatures in MORSE.
    """


    def __init__(self, obj, parent=None):
        """
        Constructor method.
        Receives the reference to the Blender object.
        """     
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)
        logger.info('Component initialized')
        
    @service
    def test_service(self):
        """
        MORSE Service to test the services.
        """
        print("\ntest_service:")
        print("DIR self:\n" + str(dir(self)))

        print("\nname: " + str(self.name))

        print("\nblender_obj: " + str(self.blender_obj))
        print("blender_obj type: " + str(type(self.blender_obj)))
        print("DIR blender_obj:\n" + str(dir(self.blender_obj)))

        print("\nchannels: " + str(self.blender_obj.channels))
        print("DIR channels[0]:\n" + str(dir(self.blender_obj.channels[0])))

        print("\nconstraints: " + str(self.blender_obj.constraints))
        # print("DIR constraints[0]:\n" + str(dir(self.blender_obj.constraints[0])))
        # print("constraints[0].active: " + str(self.blender_obj.constraints[0].active))
        # print("constraints[0].target: " + str(self.blender_obj.constraints[0].target))
        # self.blender_obj.constraints[0].target = None
        # print("constraints[0].target: " + str(self.blender_obj.constraints[0].target))

        print("\nrobot_parent: " + str(self.robot_parent))
        print("DIR robot_parent:\n" + str(dir(self.robot_parent)))
        print("robot_parent.name: " + str(self.robot_parent.name))
        print("robot_parent.blender_obj: " + str(self.robot_parent.blender_obj))
        print("DIR robot_parent.blender_obj:\n" + str(dir(self.robot_parent.blender_obj)))
        print("robot_parent.blender_obj.name: " + str(self.robot_parent.blender_obj.name))

        return None

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
        channel = armature.channels[str(channel_name)]
        # get the rotation in xyz
        rotation = channel.joint_rotation.to_tuple()

        return rotation

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
        channel = armature.channels[str(channel_name)]
        self.set_joint_rotation(armature, channel, rotation)
        return None

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