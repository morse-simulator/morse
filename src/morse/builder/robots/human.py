import logging; logger = logging.getLogger("morserobots." + __name__)
import os
from morse.builder import AbstractComponent, MORSE_COMPONENTS

class Human(AbstractComponent):
    """ Append a human model to the scene.

    The human model currently available in MORSE comes with its
    own subjective camera and several features for object manipulation.

    It also exposes a :doc:`human posture component <morse/user/sensors/human_posture>`
    that can be accessed by the ``armature`` member.

    Usage example:

    .. code-block:: python

       #! /usr/bin/env morseexec

       from morse.builder import *

       human = Human()
       human.translate(x=5.5, y=-3.2, z=0.0)
       human.rotate(z=-3.0)

       human.armature.configure_mw('pocolibs',
                        ['Pocolibs',
                         'export_posture',
                         'morse/middleware/pocolibs/sensors/human_posture',
                         'human_posture'])

    Currently, only one human per simulation is supported.
    """
    def __init__(self, style=None):
        """ The 'style' parameter is only to switch to the mocap_human file. """
        AbstractComponent.__init__(self, category='robots')
        if style == 'ik_human':
            self._blendname = 'mocap_human'
        elif style == 'mocap_human':
            self._blendname = 'mocap_human'
        else:
            self._blendname = 'human'

        self.append_meshes()

        self.set_blender_object(self.get_selected("Human"))

        self.armature = None

        try:
            obj = self.get_selected("HumanArmature")
            self.armature = AbstractComponent(obj, "human_posture")
        except KeyError:
            logger.error("Could not find the human armature! (I was looking " +\
                         "for an object called 'HumanArmature' in the 'Human'" +\
                         " children). I won't be able to export the human pose" +\
                         " to any middleware.")

        # IK human has no object called Hips_Empty, so avoid this step
        if not style:
            # fix for Blender 2.6 Animations
            if obj:
                hips = self.get_selected("Hips_Empty")
                i = 0
                for act in hips.game.actuators:
                    act.layer = i
                    i = i + 1

                i = 0
                for act in obj.game.actuators:
                    act.layer = i
                    i = i + 1

    def use_world_camera(self):
        self.properties(WorldCamera = True)

    def disable_keyboard_control(self):
        self.properties(disable_keyboard_control = True)
