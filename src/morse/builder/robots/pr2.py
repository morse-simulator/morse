import logging; logger = logging.getLogger("pr2." + __name__)

from morse.builder import Armature, Robot

class PR2(Robot):
    def __init__(self):
        Robot.__init__(self, 'pr2')
        self.properties(Class = "PR2Class", \
                        Path = "morse/robots/pr2", \
                        COLOR = "0.0, 0.0, 1.0")

        self.head = None
        self.l_arm = None
        self.r_arm = None
        self.torso_lift = None

        try:
            self.head = Armature("head_armature")
            self.append(self.head)
            self.l_arm = Armature("l_arm_controller")
            self.append(self.l_arm)
            self.r_arm = Armature("r_arm_controller")
            self.append(self.r_arm)
            self.torso_lift = Armature("torso_lift_armature")
            self.append(self.torso_lift)
        except KeyError:
            logger.error("Could not find the PR2 head armature! (I was " +\
                         "looking for an object called 'head_armature' in " +\
                         "'pr2' children). I won't be able to export the PR2" +\
                         " head pose to any middleware.")

    def set_color(self, color = (0.0, 0.0, 0.8)):
        #set the head color
        self.get_child("head_tilt_link").material_slots['HeadTilt'].material.node_tree.nodes['Material'].material.diffuse_color = color

