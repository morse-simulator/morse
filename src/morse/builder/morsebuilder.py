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
                print ("Unable to open file '%s'. Exception: %s" % (filepath, detail))

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

class Sensor(Component):
    def __init__(self, name):
        Component.__init__(self, 'sensors', name)

class Actuator(Component):
    def __init__(self, name):
        Component.__init__(self, 'actuators', name)

class Middleware(Component):
    def __init__(self, name):
        Component.__init__(self, 'middleware', name)

class Modifier(Component):
    def __init__(self, name):
        Component.__init__(self, 'modifiers', name)

class Environment(Component):
    def __init__(self, name):
        Component.__init__(self, 'environments', name)
        self._created = False
        self._camera_location = [5, -5, 5]
        self._camera_rotation = [0.7854, 0, 0.7854]
        self._environment_file = name

    def _write(self):
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
        Expected argument is a list with the desired position for the camera """
        self._camera_location = location

    def aim_camera(self, rotation):
        """ Store the orientation that will be givent to the default camera
        Expected argument is a list with the desired orientation for the camera """
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

    def __del__(self):
        """ Call the create method if the user has not explicitly called it """
        if not self._created:
            self.create()
