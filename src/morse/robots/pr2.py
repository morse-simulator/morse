# @Authors: Peter Roelants peter.roelants@gmail.com
# @Owner: KU Leuven - Dep. Mechanical Eng. - Robotics
# @File: pr2.py
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
import morse.core.robot
from morse.core.services import service


class PR2Class(morse.core.robot.MorseRobotClass):
    """ 
    Class definition for the PR2.
    Sub class of Morse_Object.

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

        self.armatures = []
        # Search armatures and torso in all objects parented to the pr2 empty
        for obj in self.blender_obj.childrenRecursive:
            # Check if obj is an armature
            if type(obj).__name__ == 'BL_ArmatureObject':
                self.armatures.append(obj.name)
            if obj.name == 'torso_lift_joint':
                self.torso = obj

        # constant that holds the original height of the torso from the ground
        # These values come from the pr2 urdf file
        self.TORSO_BASE_HEIGHT = (0.739675 + 0.051)
        self.TORSO_LOWER = 0.0  # lower limit on the torso z-translantion
        self.TORSO_UPPER = 0.31  # upper limit on the torso z-translation
        
        logger.info('Component initialized')

    
    @service
    def get_armatures(self):
        """
        MORSE Service that returns a list of all the armatures on the PR2 robot.
        """
        return self.armatures

    @service
    def set_torso(self, height):
        """
        MORSE Service that sets the z-translation of the torso to original_z + height.
        """
        if self.TORSO_LOWER < height < self.TORSO_UPPER:
            self.torso.localPosition = [-0.05, 0, self.TORSO_BASE_HEIGHT + height]
            return "New torso z position: " + str(self.torso.localPosition[2])
        else:
            return "Not a valid height, value has to be between 0.0 and 0.31!"
            
    @service
    def get_torso(self):
        """
        MORSE Service that returns the z-translation of the torso.
        """
        return self.torso.localPosition[2] - self.TORSO_BASE_HEIGHT

    @service
    def get_torso_minmax(self):
        """
        MORSE Service that returns the minimum an maximum z-translation that the torso can make from the base.
        Returns a list [min,max]
        """
        return [self.TORSO_LOWER, self.TORSO_UPPER]


    def default_action(self):
        """
        Main function of this component.
        """
        pass
