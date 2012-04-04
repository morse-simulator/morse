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
            head_obj = self._get_selected("head_armature")
            l_arm_obj = self._get_selected("l_arm_controller")
            r_arm_obj = self._get_selected("r_arm_controller")
            torso_lift_obj = self._get_selected("torso_lift_armature")
            self._head_tilt_link = self._get_selected("head_tilt_link")

            self.head = AbstractComponent(head_obj, "head_armature")
            self.l_arm = AbstractComponent(l_arm_obj, "l_arm_controller")
            self.r_arm = AbstractComponent(r_arm_obj, "r_arm_controller")
            self.torso_lift = AbstractComponent(torso_lift_obj, "torso_lift_armature")
            
        except KeyError:
            logger.error("Could not find the PR2 head armature! (I was looking " +\
                         "for an object called 'head_armature' in 'pr2'" +\
                         " children). I won't be able to export the PR2 head pose" +\
                         " to any middleware.")

    def set_color(self, color = (0.0, 0.0, 0.8)):
        #set the head color
        self._head_tilt_link.material_slots['HeadTilt'].material.node_tree.nodes['Material'].material.diffuse_color = color

