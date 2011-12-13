import logging; logger = logging.getLogger("pr2extension." + __name__)

from morse.builder.morsebuilder import *

class PR2(Robot):
    def __init__(self):
        Component.__init__(self, 'robots', 'pr2/pr2_25_morse')
        
        self.head = None
        self.l_arm = None
        self.r_arm = None
        
        try:
            self._blendobj = bpy.context.selected_objects[0]            
            # !!! WARNING (TODO)!!!! If we import the armature like this, it will NOT work if there are more than one PR2!
            # if we incude a second PR2, the armatures will be named "head_armature.001" etc.
            head_obj = bpy.data.objects["head_armature"]
            l_arm_obj = bpy.data.objects["l_arm_armature"]
            r_arm_obj = bpy.data.objects["r_arm_armature"]
            
            self.head = AbstractComponent(head_obj, "head_armature")
            self.l_arm = AbstractComponent(l_arm_obj, "l_arm_armature")
            self.r_arm = AbstractComponent(r_arm_obj, "r_arm_armature")
            
        except KeyError:
            logger.error("Could not find the PR2 head armature! (I was looking " +\
                         "for an object called 'head_armature' in 'pr2'" +\
                         " children). I won't be able to export the PR2 head pose" +\
                         " to any middleware.")
