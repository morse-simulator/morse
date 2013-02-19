import logging; logger = logging.getLogger("pr2." + __name__)

from morse.builder import *
from morse.builder.sensors import *
from morse.builder.actuators import *

class BarePR2(Robot):
    """
    A PR2 model, without any sensor or actuator.
    """
    def __init__(self):
        Robot.__init__(self, 'pr2')
        self.properties(classpath = "morse.robots.pr2.PR2Class", \
                        COLOR = "0.0, 0.0, 1.0")

    def set_color(self, color = (0.0, 0.0, 0.8)):
        """
        Allows to change the PR2 head color.
        """
        self.get_child("head_tilt_link").material_slots['HeadTilt'].material.node_tree.nodes['Material'].material.diffuse_color = color

class BasePR2(BarePR2):
    """
    A PR2 only equipped with its armatures for the arms, the torso and the
    head.

    It also provides the compound sensor ``pr2.joint_state`` that exports the
    whole joint state of the robot.

    """
    def __init__(self):
        BarePR2.__init__(self)

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


    def add_interface(self, interface):
        if interface == "socket":
            self.joint_state.add_stream("socket", "morse.middleware.sockets.jointstate.PR2JointStatePublisher")
            self.torso.add_service('socket')
            self.head.add_service('socket')
            self.l_arm.add_service('socket')
            self.r_arm.add_service('socket')
        elif interface == "ros":

            self.joint_state.add_stream("ros", 
                                        "morse.middleware.ros.jointstate.JointStatePR2Publisher",
                                        topic = "/joint_states")
            self.joint_state.add_service("ros")



            self.torso_pose.add_stream("ros",
                        "morse.middleware.ros.jointtrajectorycontrollers.JointTrajectoryControllerStatePublisher",
                        topic="/torso_lift_controller/state")
            self.torso.configure_overlay("ros",
              "morse.middleware.ros.overlays.armatures.ArmatureController",
              namespace = "/torso_lift_controller")



            self.head_pose.add_stream("ros",
                       "morse.middleware.ros.jointtrajectorycontrollers.JointTrajectoryControllerStatePublisher",
                       topic="/head_controller/state")
            self.head.configure_overlay("ros",
              "morse.middleware.ros.overlays.armatures.ArmatureController",
              namespace = "/head_controller")


            self.l_arm_pose.add_stream("ros",
                      "morse.middleware.ros.jointtrajectorycontrollers.JointTrajectoryControllerStatePublisher",
                      topic="/l_arm_controller/state")
            self.l_arm.configure_overlay("ros",
              "morse.middleware.ros.overlays.armatures.ArmatureController",
              namespace = "/l_arm_controller")


            self.r_arm_pose.add_stream("ros", 
                 "morse.middleware.ros.jointtrajectorycontrollers.JointTrajectoryControllerStatePublisher",
                 topic="/r_arm_controller/state")
            self.r_arm.configure_overlay("ros",
              "morse.middleware.ros.overlays.armatures.ArmatureController",
              namespace = "/r_arm_controller")


class LocalizedPR2(BasePR2):
    def __init__(self, with_keyboard = True, show_laser = False):
        BasePR2.__init__(self)

        ###################################
        # Actuators
        ###################################


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

        self.pose = Pose()
        self.append(self.pose)

    def add_interface(self, interface):

        super(self.__class__, self).add_interface(interface)

        if interface == "ros": 
            self.motion.add_stream("ros", topic="/cmd_vel")
            self.pose.add_stream("ros", method="morse.middleware.ros.pose.TFPublisher")


class NavPR2(BasePR2):
    """
    A PR2 equipped with sensors and actuators required for 2D navigation.

    """
    def __init__(self, with_keyboard = True, show_laser = False):
        BasePR2.__init__(self)

        ###################################
        # Actuators
        ###################################


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

        super(self.__class__, self).add_interface(interface)

        if interface == "socket":
            pass
        elif interface == "ros": 
            self.base_scan.add_stream("ros", topic="/scan")
            self.odometry.add_stream("ros", topic="/odom")
            self.motion.add_stream("ros", topic="/cmd_vel")


