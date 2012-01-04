from morse.middleware.ros_request_manager import ros_service
from morse.core.overlay import MorseOverlay
from morse.core import status

import roslib; roslib.load_manifest('morsetesting')
from morsetesting.srv import *

class PR2(MorseOverlay):

    def __init__(self, overlaid_object):
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(overlaid_object)
    
    @ros_service(type = GetHead)
    def get_head(self):
        #print("Head rotations: %s"%self.overlaid_object.get_rotations())
        return self.overlaid_object.get_rotations()
        
    @ros_service(type = GetTorso)
    def get_torso(self):
        # NOTE: We do NOT care about the IK limits here!
        return self.overlaid_object.get_translation("torso_lift")[1]
        
    @ros_service(type = SetTorso)
    def set_torso(self, height):
        if height >= 0 and height <= 0.311:
            self.overlaid_object.set_location("torso_lift", [0, height, 0])
            return True
        else: 
            print("Received invalid value: %s for PR2 torso. Torso height must be betweeen 0 and 0.31"%height)
            return False
        
    @ros_service(type = SetHead)
    def set_head(self, pan, tilt):
        #print("Setting head to: %s"%pan, tilt)
        self.overlaid_object.set_rotation("head_pan", [0, pan, 0])
        self.overlaid_object.set_rotation("head_tilt", [0, 0, tilt])
        return True
        
    @ros_service(type = TuckLeftArm)    
    def tuck_left_arm(self):
        #print("Channels are %s"%self.overlaid_object.get_channels())
        self.overlaid_object.set_rotation("l_shoulder_pan", [0, 0, -0.06024])
        self.overlaid_object.set_rotation("l_shoulder_lift", [-1.248526, 0, 0])
        self.overlaid_object.set_rotation("l_upper_arm", [0, 1.78907, 0])
        self.overlaid_object.set_rotation("l_elbow", [1.683386, 0, 0])
        self.overlaid_object.set_rotation("l_forearm", [0, 1.7343417, 0])
        self.overlaid_object.set_rotation("l_wrist_flex", [0.0962141, 0, 0])
        self.overlaid_object.set_rotation("l_wrist_roll", [0, 0.0864407, 0])
        return True
        
    @ros_service(type = TuckRightArm)    
    def tuck_right_arm(self):
        #print("Channels are %s"%self.overlaid_object.get_channels())
        self.overlaid_object.set_rotation("r_shoulder_pan", [0, 0, 0.023593])
        self.overlaid_object.set_rotation("r_shoulder_lift", [-1.1072800, 0, 0])
        self.overlaid_object.set_rotation("r_upper_arm", [0, 1.5566882, 0])
        self.overlaid_object.set_rotation("r_elbow", [-2.124408, 0, 0])
        self.overlaid_object.set_rotation("r_forearm", [0, 1.4175, 0])
        self.overlaid_object.set_rotation("r_wrist_flex", [1.8417, 0, 0])
        self.overlaid_object.set_rotation("r_wrist_roll", [0, -0.21436, 0])
        return True
