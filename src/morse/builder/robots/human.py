import logging; logger = logging.getLogger("morserobots." + __name__)
from morse.builder import bpymorse
from morse.builder import GroundRobot
from morse.builder.sensors import ArmaturePose
from morse.builder.creator import ArmatureCreator

class Human(GroundRobot):
    """ Append a human model to the scene.

    The human model currently available in MORSE comes with its
    own subjective camera and several features for object manipulation.

    It also exposes a :doc:`human posture component <../sensors/human_posture>`
    that can be accessed by the ``armature`` member.

    Usage example:

    .. code-block:: python

       #! /usr/bin/env morseexec

       from morse.builder import *

       human = Human()
       human.translate(x=5.5, y=-3.2, z=0.0)
       human.rotate(z=-3.0)

       human.armature.add_stream('pocolibs')

    Currently, only one human per simulation is supported.
    """
    def __init__(self, filename='human', name = None):
        """ The 'style' parameter is only to switch to the mocap_human file.

        :param filename: 'human' (default) or 'mocap_human'
        """
        GroundRobot.__init__(self, filename, name)

        self.suffix = self.name[-4:] if self.name[-4] == "." else ""

        self.armature = None
        if filename == 'mocap_human':
            self.properties(classpath="morse.robots.mocap_human.MocapHuman")
        else:
            self.properties(classpath="morse.robots.human.Human")

        try:
            self.armature = ArmatureCreator(armature_name = "HumanArmature" + self.suffix)
            self.append(self.armature)
        except KeyError:
            logger.error("Could not find the human armature! (I was looking " +\
                         "for an object called 'HumanArmature' in the human" +\
                         " children). I won't be able to export the human pose" +\
                         " to any middleware.")

        # Add an armature sensor. "joint_stateS" to match standard ROS spelling.
        self.joint_states = ArmaturePose()
        self.armature.append(self.joint_states)

        # fix for Blender 2.6 Animations
        armature_object = self.get_child(self.armature.name)
        if armature_object:
            hips = self.get_child("Hips_Empty")
            # IK human has no object called Hips_Empty, so avoid this step
            if hips:
                for i, actuator in enumerate(hips.game.actuators):
                    actuator.layer = i
                for i, actuator in enumerate(armature_object.game.actuators):
                    actuator.layer = i

    def after_renaming(self):
        if self._blender_filename == 'mocap_human':
            # no need for mocap
            return

        # Store the human real name (ie, after renaming) in its link 'POS_EMPTY' and 'Human_Camera' object, for later control.

        pos_empty = bpymorse.get_object("POS_EMPTY" + self.suffix)
        bpymorse.select_only(pos_empty)

        bpymorse.new_game_property()
        prop = pos_empty.game.properties
        # select the last property in the list (which is the one we just added)
        prop[-1].name = "human_name"
        prop[-1].type = "STRING"
        prop[-1].value = self.name

        human_camera = bpymorse.get_object("Human_Camera" + self.suffix)
        bpymorse.select_only(human_camera)

        bpymorse.new_game_property()
        prop = human_camera.game.properties
        # select the last property in the list (which is the one we just added)
        prop[-1].name = "human_name"
        prop[-1].type = "STRING"
        prop[-1].value = self.name

    def add_interface(self, interface):
        if interface == "socket":
            self.joint_states.add_stream("socket")
            self.armature.add_service('socket')

        elif interface == "ros":

            self.joint_states.add_stream("ros")

            self.armature.add_service("ros")
            self.armature.add_overlay("ros",
              "morse.middleware.ros.overlays.armatures.ArmatureController")

        elif interface == "pocolibs":
            self.armature.properties(classpath="morse.sensors.human_posture.HumanPosture")
            self.add_stream(interface)


    def use_world_camera(self):
        self.properties(WorldCamera = True)

    def disable_keyboard_control(self):
        self.properties(disable_keyboard_control = True)
