# @Authors: David Hodo hododav@tigermail.auburn.edu
# @Owner: Auburn University
# @File: steer_force.py
# @Description:  
# @License: 
# (C) 2010 Name, David Hodo
# Auburn University, USA
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
import morse.core.actuator

class SteerForceActuatorClass(morse.core.actuator.MorseActuatorClass):
    """ Motion controller using engine force and steer angle speeds

    This class will read engine force and steer angle (steer, force)
    as input from an external middleware, and then apply them
    to the parent robot.
    """

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self.local_data['steer'] = 0.0
        self.local_data['force'] = 0.0
        self.local_data['brake'] = 0.0
        
        logger.info('Component initialized')


    def default_action(self):
        """ Apply (steer, force) to the parent robot. """
        # Get the Blender object of the parent robot
        parent = self.robot_parent
        
        # Update the steering value for these wheels:
        # The number at the end represents the wheel 'number' in the 
        #  order they were created when initializing the robot.
        # Front wheels #0 and #1.
        # Rear wheels #2 and #3.
        parent.vehicle.setSteeringValue(self.local_data['steer'],0)
        parent.vehicle.setSteeringValue(self.local_data['steer'],1)

        # Update the Force (speed) for these wheels:
        parent.vehicle.applyEngineForce(self.local_data['force']*.4,0)
        parent.vehicle.applyEngineForce(self.local_data['force']*.4,1)
        parent.vehicle.applyEngineForce(self.local_data['force']*.4,2)
        parent.vehicle.applyEngineForce(self.local_data['force'] *.4,3)

        # Brakes:
        # Applies the braking force to each wheel listed:
        # ['brakes'] = the game property value for the car labeled 'brakes'
        # Default value is 0:
        parent.vehicle.applyBraking(self.local_data['brake']*.1,0)
        parent.vehicle.applyBraking(self.local_data['brake']*.1,1)
        parent.vehicle.applyBraking(self.local_data['brake']*1.3,2)
        parent.vehicle.applyBraking(self.local_data['brake']*1.3,3)
