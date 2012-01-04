import logging; logger = logging.getLogger("pr2extension." + __name__)

from morse.builder.morsebuilder import *

class PR2(Robot):
    def __init__(self):
        Component.__init__(self, 'robots', 'pr2')

        self.head = None
        self.l_arm = None
        self.r_arm = None
        self.torso_lift = None
        
        try:
            self._blendobj = bpy.context.selected_objects[0]            
            # !!! WARNING (TODO)!!!! If we import the armature like this, it will NOT work if there are more than one PR2!
            # if we incude a second PR2, the armatures will be named "head_armature.001" etc.
            head_obj = bpy.data.objects["head_armature"]
            l_arm_obj = bpy.data.objects["l_arm_armature"]
            r_arm_obj = bpy.data.objects["r_arm_armature"]
            torso_lift_obj = bpy.data.objects["torso_lift_armature"]

            self.head = AbstractComponent(head_obj, "head_armature")
            self.l_arm = AbstractComponent(l_arm_obj, "l_arm_armature")
            self.r_arm = AbstractComponent(r_arm_obj, "r_arm_armature")
            self.torso_lift = AbstractComponent(torso_lift_obj, "torso_lift_armature")
            
        except KeyError:
            logger.error("Could not find the PR2 head armature! (I was looking " +\
                         "for an object called 'head_armature' in 'pr2'" +\
                         " children). I won't be able to export the PR2 head pose" +\
                         " to any middleware.")

    def set_color(self, color = (0.0, 0.0, 0.8)):
        #set the head color
        bpy.data.objects['head_tilt_link'].material_slots['HeadTilt'].material.node_tree.nodes['Material'].material.diffuse_color = color


