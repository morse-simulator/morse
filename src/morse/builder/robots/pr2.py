import logging; logger = logging.getLogger("pr2." + __name__)

from morse.builder import *
from morse.builder.sensors import *
from morse.builder.actuators import *


class PR2(Robot):
    def __init__(self, with_keyboard = True, show_laser = False):
        Robot.__init__(self, 'pr2')
        self.properties(Class = "PR2Class", \
                        Path = "morse/robots/pr2", \
                        COLOR = "0.0, 0.0, 1.0")

        ###################################
        # Actuators
        ###################################

        # Armatures and armature pose sensors
        # The armatures are already present in the PR2 blender model.

         # torso
        self.torso = Armature("torso_armature")
        self.torso_pose = ArmaturePose()
        self.torso_pose.name = "pose"
        self.torso.append(self.torso_pose)
        self.append(self.torso)

        # head
        self.head = Armature("head_armature")
        self.head_pose = ArmaturePose()
        self.head_pose.name = "pose"
        self.head.append(self.head_pose)
        self.torso.append(self.head)

        # arms
        self.l_arm = Armature("l_arm_armature")
        self.l_arm_pose = ArmaturePose()
        self.l_arm_pose.name = "pose"
        self.l_arm.append(self.l_arm_pose)
        self.torso.append(self.l_arm)

        self.r_arm = Armature("r_arm_armature")
        self.r_arm_pose = ArmaturePose()
        self.r_arm_pose.name = "pose"
        self.r_arm.append(self.r_arm_pose)
        self.torso.append(self.r_arm)

        # joint state
        self.joint_state = CompoundSensor([self.torso_pose, self.head_pose, self.l_arm_pose, self.r_arm_pose])
        self.append(self.joint_state)

        # Motion controller
        self.motion = MotionXYW()
        self.append(self.motion)

        # (optionally) keyboard controller
        if with_keyboard:
            keyboard = Keyboard()
            self.append(keyboard)

        ###################################
        # Sensors
        ###################################

        # Odometry
        self.odometry = Odometry()
        self.append(self.odometry)

        # Base laser scanner
        self.base_scan = Hokuyo()
        self.base_scan.translate(x=0.275, z=0.252)
        self.append(self.base_scan)
        self.base_scan.properties(Visible_arc = show_laser)
        self.base_scan.properties(laser_range = 30.0)
        self.base_scan.properties(resolution = 1.0)
        self.base_scan.properties(scan_window = 180.0)
        self.base_scan.create_laser_arc()


        ###################################
        ###################################

        logger.info("PR2 created.")

    def add_interface(self, interface):
        if interface == "socket":
            self.joint_state.add_stream("socket", "post_pr2_jointstate", "morse/middleware/sockets/jointstate")
        elif interface == "ros":
            self.joint_state.add_stream("ros", "post_pr2_jointstate", "morse/middleware/ros/jointstate", topic = "/joint_state")
            #self.torso.add_stream("ros", topic="/torso_controller/command")
            self.torso_pose.add_stream("ros", topic="/torso_controller/state")

            #self.head.add_stream("ros", topic="/head_controller/command")
            self.head_pose.add_stream("ros", topic="/head_controller/state")
            
            #self.l_arm.add_stream("ros", topic="/l_arm_controller/command")
            self.l_arm_pose.add_stream("ros", topic="/l_arm_controller/state")
            
            #self.r_arm.add_stream("ros", topic="/r_arm_controller/command")
            self.r_arm_pose.add_stream("ros", topic="/r_arm_controller/state")

    def set_color(self, color = (0.0, 0.0, 0.8)):
        #set the head color
        self.get_child("head_tilt_link").material_slots['HeadTilt'].material.node_tree.nodes['Material'].material.diffuse_color = color


