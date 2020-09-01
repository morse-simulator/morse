# ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ Mission Systems Pty Ltd ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Projects: WamV-Morse-Sim
# 
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Primary Author:
# david battle <david.battle@missionsystems.com.au>
# Other Author(s):
# none
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Date Created:
# 29/01/2019
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
from .abstract_modifier import AbstractModifier
from math import pi

class Bluefin21PoseModifier(AbstractModifier):
    def initialize(self):
        """ initialization of parameters ... """
        
    def modify(self):
        # Place where the data modification occurs
        self.data['yaw']   =  self.data['yaw'] - pi / 2.0
        self.data['pitch'] = -self.data['pitch']

class WamVPoseModifier(AbstractModifier):
    def initialize(self):
        """ initialization of parameters ... """
        
    def modify(self):
        # Place where the data modification occurs
        self.data['yaw'] = self.data['yaw'] - pi/2.0
        