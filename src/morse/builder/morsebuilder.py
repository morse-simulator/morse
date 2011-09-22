import logging; logger = logging.getLogger("morsebuilder." + __name__)
import os
import bpy
from morse.builder.abstractcomponent import *

"""
Morse Builder API

To test this module you can c/p the following code in Blender Python console::

import sys
sys.path.append("/usr/local/lib/python3/dist-packages")
from morse.builder.morsebuilder import *
atrv=Robot("atrv")

The string passed to the differents Components Classes must be an existing 
.blend file-name, ie. for ``Robot("atrv")`` the file ``atrv.blend`` must exists 
in the folder ``MORSE_COMPONENTS/robots/``.
"""

class PassiveObject(AbstractComponent):
    """ Allows to import any Blender object to the scene.
    """

    def __init__(self, file, object = None, keep_pose = False):
        """
        :param blenderfile: The Blender file to load. Path can be absolute
                           or relative to MORSE assets' installation path
                           (typically, $PREFIX/share/data/morse)
        :param object: (optional) the name of the object to load in the 
                       Blender file. If not set, all objects present in the file
                       are loaded.
        :param keep_pose: If set, the object pose (translation and rotation)
                        in the Blender file is kept. Else, the object 
                        own center is placed at origin and all rotation are
                        reset.
        :return: a new AbstractComponent instance.
        """
        AbstractComponent.__init__(self)
        if os.path.exists(file):
            filepath = file
        else:
            filepath = os.path.join(MORSE_COMPONENTS,file)
            if not os.path.exists(filepath):
                logger.error("Blender file %s for external asset import can not be found.\n" + \
                         "Either provide an absolute path, or a path relative to MORSE \n" + \
                         "asset directory (typically $PREFIX/share/data/morse)" % (file))

        with bpy.data.libraries.load(filepath) as (src, _):
            if object:
                objlist = [{'name':object}]
            else:
                try:
                    objlist = [{'name':obj} for obj in src.objects]
                except UnicodeDecodeError as detail:
                    logger.error("Unable to open file '%s'. Exception: %s" % (filepath, detail))

        print("objlist %s" % (objlist))

        bpy.ops.object.select_all(action='DESELECT')
        bpy.ops.wm.link_append(directory=filepath + '/Object/', link=False, 
                autoselect=True, files=objlist)
        # here we use the fact that after appending, Blender select the objects 
        # and the root (parent) object first ( [0] )
        self._blendobj = bpy.context.selected_objects[0]
        
        if not keep_pose:
            self._blendobj.location = (0.0,0.0,0.0)
            self._blendobj.rotation_euler = (0.0,0.0,0.0)


class Component(AbstractComponent):
    """ Append a morse-component to the scene

    cf. `bpy.ops.wm.link_append 
    <http://www.blender.org/documentation/blender_python_api_2_59_release/bpy.ops.wm.html#bpy.ops.wm.link_append>`_ 
     and 
    `bpy.data.libraries.load 
    <http://www.blender.org/documentation/blender_python_api_2_59_release/bpy.types.BlendDataLibraries.html>`_ 
    """
    def __init__(self, category, name):
        AbstractComponent.__init__(self)
        filepath = os.path.join(MORSE_COMPONENTS, category, name + '.blend')

        with bpy.data.libraries.load(filepath) as (src, _):
            try:
                objlist = [{'name':obj} for obj in src.objects]
            except UnicodeDecodeError as detail:
                logger.error("Unable to open file '%s'. Exception: %s" % (filepath, detail))

        bpy.ops.object.select_all(action='DESELECT')
        bpy.ops.wm.link_append(directory=filepath + '/Object/', link=False, 
                autoselect=True, files=objlist)
        self._blendname = name # for middleware dictionary
        # here we use the fact that after appending, Blender select the objects 
        # and the root (parent) object first ( [0] )
        self._blendobj = bpy.context.selected_objects[0]

class Robot(Component):
    def __init__(self, name):
        Component.__init__(self, 'robots', name)
    def make_external(self):
        self._blendobj.game.properties['Robot_Tag'].name = 'External_Robot_Tag'


class Sensor(Component):
    def __init__(self, name):
        Component.__init__(self, 'sensors', name)

class Actuator(Component):
    def __init__(self, name):
        Component.__init__(self, 'actuators', name)

class Modifier(Component):
    def __init__(self, name):
        Component.__init__(self, 'modifiers', name)

class Environment(Component):
    """ Class to configure the general environment of the simulation
    It handles the scenario file, general properties of the simulation,
    the default location and orientation of the camera, the Blender GE settings
    and also writes the 'component_config.py' file.
    """
    def __init__(self, name):
        Component.__init__(self, 'environments', name)
        self._created = False
        self._camera_location = [5, -5, 5]
        self._camera_rotation = [0.7854, 0, 0.7854]
        self._environment_file = name

    def _write(self):
        """ Write the 'component_config.py' file with the supplied settings """
        if not 'component_config.py' in bpy.data.texts.keys():
            bpy.ops.text.new()
            bpy.data.texts[-1].name = 'component_config.py'
        cfg = bpy.data.texts['component_config.py']
        cfg.clear()
        cfg.write('component_mw = ' + json.dumps(cfg_middleware, indent=1) )
        cfg.write('\n')
        cfg.write('component_modifier = ' + json.dumps(cfg_modifier, indent=1) )
        cfg.write('\n')
        cfg.write('component_service = ' + json.dumps(cfg_service, indent=1) )
        cfg.write('\n')
        cfg.write('overlays = ' + json.dumps(cfg_overlay, indent=1) )
        cfg.write('\n')

    def place_camera(self, location):
        """ Store the position that will be givent to the default camera
        Expected argument is a list with the desired position for the camera
        """
        self._camera_location = location

    def aim_camera(self, rotation):
        """ Store the orientation that will be givent to the default camera
        Expected argument is a list with the desired orientation for the camera
        """
        self._camera_rotation = rotation

    def create(self):
        """ Generate the scene configuration and insert necessary objects
        """
        # Insert modifiers into the scene
        for mod in scene_modifiers:
            Modifier(mod)
        # Write the configuration of the middlewares
        self._write()
        # Add the necessary objects
        base = Component('props', 'basics')
        # Write the name of the 'environment file'
        ssh = bpy.data.objects['Scene_Script_Holder']
        ssh.game.properties['environment_file'].value = self._environment_file
        # Set the position of the camera
        camera_fp = bpy.data.objects['CameraFP']
        camera_fp.location = self._camera_location
        camera_fp.rotation_euler = self._camera_rotation
        # Make CameraFP the active camera
        bpy.ops.object.select_all(action = 'DESELECT')
        bpy.ops.object.select_name(name = 'CameraFP')
        self._created = True

    def show_debug_properties(self, value):
        if isinstance(value, bool):
            bpy.data.scenes[0].game_settings.show_debug_properties = value

    def show_framerate(self, value):
        if isinstance(value, bool):
            bpy.data.scenes[0].game_settings.show_framerate_profile = value

    def show_physics(self, value):
        if isinstance(value, bool):
            bpy.data.scenes[0].game_settings.show_physics_visualization = value

    def configure_node(self, protocol='socket', node_name='node', server_address='localhost', server_port='65000'):
        """ Write the 'multinode_config.py' script """
        node_config = { 'protocol': protocol,
                        'node_name': node_name,
                        'server_address': server_address,
                        'server_port': server_port,}
        # Create the config file if it does not exist
        if not 'multinode_config.py' in bpy.data.texts.keys():
            bpy.ops.text.new()
            bpy.data.texts[-1].name = 'multinode_config.py'
        cfg = bpy.data.texts['multinode_config.py']
        cfg.clear()
        cfg.write('node_config = ' + json.dumps(node_config, indent=1) )
        cfg.write('\n')
        

    def __del__(self):
        """ Call the create method if the user has not explicitly called it """
        if not self._created:
            self.create()
