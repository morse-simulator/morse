# usage: blender -P {file}
import os, sys, subprocess

def ext_exec(cmd, python=None):
    if not python:
        python = 'python%i.%i'%(sys.version_info.major, sys.version_info.minor)
    return subprocess.getoutput('%s -c"%s"' % (python, cmd) )

def fix_python_path(python=None):
    pythonpath = ext_exec("import os,sys;print(os.pathsep.join(sys.path))")
    sys.path.extend(pythonpath.split(os.pathsep))

# add system Python ``sys.path`` to Blender Python ``sys.path``
# allow us to import ``morse`` package (installed in system Python path)
fix_python_path()

import bpy
from morse.builder import AbstractComponent
from morse.builder.bpymorse import *
from mathutils import Vector, Euler

def init_morse():
    setup_scene()
    setup_basics()
    finalize()

def finalize():
    camera = bpy.data.objects['CameraFP']
    camera.data.clip_end = 1000
    bpy.context.scene.camera = camera
    # see the texture in realtime (in the 3D viewport)
    set_viewport('TEXTURED', 10000)

def setup_scene(scene=None):
    """ Setup the scene """
    if not scene and bpy:
        scene = bpy.context.scene
    # Set Game mode
    scene.render.engine = 'BLENDER_GAME'
    # make sure OpenGL shading language shaders (GLSL) is the
    # material mode to use for rendering
    # scene.game_settings.material_mode = 'GLSL'
    # Set the unit system to use for button display (in edit mode) to metric
    scene.unit_settings.system = 'METRIC'
    # Select the type of Framing to Extend,
    # Show the entire viewport in the display window,
    # viewing more horizontally or vertically.
    scene.game_settings.frame_type = 'EXTEND'
    # Set the color at the horizon to dark azure
    scene.world.horizon_color = (0.05, 0.22, 0.4)
    # Display framerate and profile information of the simulation
    scene.game_settings.show_framerate_profile = True
    scene.game_settings.show_mouse = True
    scene.render.resolution_x = 320
    scene.render.resolution_y = 240
    scene.render.fps = 25

def setup_basics():
    base = AbstractComponent(category='props', filename='basics_wo_logic')
    base.append_meshes(['CameraID_text', 'Compass', 'HUD_plane', 'Keys_text', 'Screen', 'Screen_frame', 'Title_text'])
    # CAMERA : CameraFP
    bpy.ops.object.add(type='CAMERA', location=Vector((6.0, 0.0, 4.0)), rotation=Euler((1.57, 0.0, 1.57), 'XYZ'))
    bpy.context.object.name = 'CameraFP'
    bpy.context.object.game.physics_type = 'STATIC'
    bpy.context.object.hide_render = False
    properties(bpy.context.object, Sensitivity=0.001, Speed=0.1)
    game = bpy.context.object.game
    add_actuator(type='SCENE', name='Set_Camera')
    game.actuators[-1].mode = "CAMERA"
    game.actuators[-1].name = "Set_Camera"
    add_controller(type='LOGIC_AND', name='cont')
    game.controllers[-1].name = "cont"
    game.controllers['cont'].link(actuator=game.actuators['Set_Camera'])
    add_controller(type='PYTHON', name='Store degfault')
    game.controllers[-1].mode = "MODULE"
    game.controllers[-1].module = "view_camera.store_default"
    game.controllers[-1].name = "Store degfault"
    add_controller(type='PYTHON', name='MouseLook')
    game.controllers[-1].mode = "MODULE"
    game.controllers[-1].module = "view_camera.rotate"
    game.controllers[-1].name = "MouseLook"
    add_controller(type='PYTHON', name='KeyboardMove')
    game.controllers[-1].mode = "MODULE"
    game.controllers[-1].module = "view_camera.move"
    game.controllers[-1].name = "KeyboardMove"
    add_sensor(type='ALWAYS', name='onLoad')
    game.sensors[-1].name = "onLoad"
    game.sensors['onLoad'].link(game.controllers['Store degfault'])
    game.sensors['onLoad'].link(game.controllers['cont'])
    add_sensor(type='MOUSE', name='Mouse')
    game.sensors[-1].mouse_event = "MOVEMENT"
    game.sensors[-1].name = "Mouse"
    game.sensors['Mouse'].link(game.controllers['MouseLook'])
    add_sensor(type='KEYBOARD', name='All_Keys')
    game.sensors[-1].key = "NONE"
    game.sensors[-1].name = "All_Keys"
    game.sensors[-1].use_all_keys = True
    game.sensors[-1].use_pulse_true_level = True
    game.sensors['All_Keys'].link(game.controllers['KeyboardMove'])
    game.sensors['All_Keys'].link(game.controllers['MouseLook'])
    # MESH : CameraID_text
    select_only(bpy.data.objects['CameraID_text'])
    bpy.context.object.game.physics_type = 'NO_COLLISION'
    bpy.context.object.hide_render = True
    properties(bpy.context.object, Text="No camera selected")
    game = bpy.context.object.game
    # MESH : Compass
    select_only(bpy.data.objects['Compass'])
    bpy.context.object.game.physics_type = 'NO_COLLISION'
    bpy.context.object.hide_render = False
    properties(bpy.context.object, Display=True)
    game = bpy.context.object.game
    add_controller(type='PYTHON', name='Billboard')
    game.controllers[-1].mode = "MODULE"
    game.controllers[-1].module = "billboard.reset_rotation"
    game.controllers[-1].name = "Billboard"
    add_controller(type='PYTHON', name='Display')
    game.controllers[-1].mode = "MODULE"
    game.controllers[-1].module = "billboard.display"
    game.controllers[-1].name = "Display"
    add_sensor(type='ALWAYS', name='Always')
    game.sensors[-1].name = "Always"
    game.sensors[-1].use_pulse_true_level = True
    game.sensors['Always'].link(game.controllers['Billboard'])
    add_sensor(type='KEYBOARD', name='1')
    game.sensors[-1].key = "ONE"
    game.sensors[-1].modifier_key_1 = "LEFT_SHIFT"
    game.sensors[-1].name = "1"
    game.sensors['1'].link(game.controllers['Display'])
    # MESH : HUD_plane
    select_only(bpy.data.objects['HUD_plane'])
    bpy.context.object.game.physics_type = 'NO_COLLISION'
    bpy.context.object.hide_render = True
    properties(bpy.context.object, Visible=False)
    game = bpy.context.object.game
    add_actuator(type='PROPERTY', name='Toggle')
    game.actuators[-1].mode = "TOGGLE"
    game.actuators[-1].name = "Toggle"
    game.actuators[-1].object_property = ""
    game.actuators[-1].property = "Visible"
    game.actuators[-1].value = ""
    add_actuator(type='VISIBILITY', name='Show')
    game.actuators[-1].apply_to_children = True
    game.actuators[-1].name = "Show"
    game.actuators[-1].use_visible = True
    add_actuator(type='VISIBILITY', name='Hide')
    game.actuators[-1].apply_to_children = True
    game.actuators[-1].use_visible = False
    game.actuators[-1].name = "Hide"
    add_controller(type='LOGIC_AND', name='And')
    game.controllers[-1].name = "And"
    game.controllers['And'].link(actuator=game.actuators['Toggle'])
    add_controller(type='LOGIC_AND', name='And1')
    game.controllers[-1].name = "And1"
    game.controllers['And1'].link(actuator=game.actuators['Show'])
    add_controller(type='LOGIC_NAND', name='Nand')
    game.controllers[-1].name = "Nand"
    game.controllers['Nand'].link(actuator=game.actuators['Hide'])
    add_sensor(type='KEYBOARD', name='H_KEY')
    game.sensors[-1].key = "H"
    game.sensors[-1].name = "H_KEY"
    game.sensors['H_KEY'].link(game.controllers['And'])
    add_sensor(type='PROPERTY', name='Visible')
    game.sensors[-1].name = "Visible"
    game.sensors[-1].property = "Visible"
    game.sensors[-1].value = "True"
    game.sensors[-1].value_min = "True"
    game.sensors['Visible'].link(game.controllers['And1'])
    game.sensors['Visible'].link(game.controllers['Nand'])
    # MESH : Keys_text
    select_only(bpy.data.objects['Keys_text'])
    bpy.context.object.game.physics_type = 'NO_COLLISION'
    bpy.context.object.hide_render = True
    properties(bpy.context.object, Text="HUD text")
    game = bpy.context.object.game
    add_controller(type='PYTHON', name='And')
    game.controllers[-1].mode = "MODULE"
    game.controllers[-1].module = "hud_text.change_text"
    game.controllers[-1].name = "And"
    add_sensor(type='ALWAYS', name='Keyboard')
    game.sensors[-1].name = "Keyboard"
    game.sensors['Keyboard'].link(game.controllers['And'])
    # EMPTY : Scene_Script_Holder
    bpy.ops.object.add(type='EMPTY', location=Vector((0.0, 0.0, 10.828)))
    bpy.context.object.name = 'Scene_Script_Holder'
    bpy.context.object.scale = Vector((0.675, 0.675, 0.675))
    bpy.context.object.game.physics_type = 'NO_COLLISION'
    bpy.context.object.hide_render = False
    properties(bpy.context.object, paths_ok=True, UTMYOffset=0.0, UTMZOffset=0.0, environment_file="", UTMXOffset=0.0, Temperature="15.0")
    game = bpy.context.object.game
    add_actuator(type='GAME', name='Quit_sim')
    game.actuators[-1].filename = ""
    game.actuators[-1].mode = "QUIT"
    game.actuators[-1].name = "Quit_sim"
    add_actuator(type='GAME', name='Restart_sim')
    game.actuators[-1].filename = ""
    game.actuators[-1].mode = "RESTART"
    game.actuators[-1].name = "Restart_sim"
    add_controller(type='PYTHON', name='Path')
    game.controllers[-1].mode = "MODULE"
    game.controllers[-1].module = "setup_path.test"
    game.controllers[-1].name = "Path"
    game.controllers[-1].use_priority = True
    game.controllers['Path'].link(actuator=game.actuators['Quit_sim'])
    add_controller(type='PYTHON', name='Initialize')
    game.controllers[-1].mode = "MODULE"
    game.controllers[-1].module = "main.init"
    game.controllers[-1].name = "Initialize"
    game.controllers['Initialize'].link(actuator=game.actuators['Quit_sim'])
    add_controller(type='PYTHON', name='Finalize')
    game.controllers[-1].mode = "MODULE"
    game.controllers[-1].module = "main.finish"
    game.controllers[-1].name = "Finalize"
    game.controllers['Finalize'].link(actuator=game.actuators['Quit_sim'])
    add_controller(type='PYTHON', name='Switch_Camera')
    game.controllers[-1].mode = "MODULE"
    game.controllers[-1].module = "main.switch_camera"
    game.controllers[-1].name = "Switch_Camera"
    add_controller(type='LOGIC_AND', name='Fast_quit')
    game.controllers[-1].name = "Fast_quit"
    game.controllers['Fast_quit'].link(actuator=game.actuators['Quit_sim'])
    add_controller(type='PYTHON', name='Admin')
    game.controllers[-1].mode = "MODULE"
    game.controllers[-1].module = "main.simulation_main"
    game.controllers[-1].name = "Admin"
    game.controllers['Admin'].link(actuator=game.actuators['Quit_sim'])
    add_controller(type='PYTHON', name='Restart')
    game.controllers[-1].mode = "MODULE"
    game.controllers[-1].module = "main.restart"
    game.controllers[-1].name = "Restart"
    game.controllers['Restart'].link(actuator=game.actuators['Restart_sim'])
    add_controller(type='PYTHON', name='Import_path_script')
    game.controllers[-1].name = "Import_path_script"
    add_sensor(type='ALWAYS', name='onLoad')
    game.sensors[-1].name = "onLoad"
    game.sensors['onLoad'].link(game.controllers['Path'])
    add_sensor(type='PROPERTY', name='paths_ok')
    game.sensors[-1].name = "paths_ok"
    game.sensors[-1].property = "paths_ok"
    game.sensors[-1].value = "True"
    game.sensors[-1].value_min = "True"
    game.sensors['paths_ok'].link(game.controllers['Initialize'])
    add_sensor(type='KEYBOARD', name='ESC_KEY')
    game.sensors[-1].key = "ESC"
    game.sensors[-1].name = "ESC_KEY"
    game.sensors['ESC_KEY'].link(game.controllers['Finalize'])
    add_sensor(type='KEYBOARD', name='F9_KEY')
    game.sensors[-1].key = "F9"
    game.sensors[-1].name = "F9_KEY"
    game.sensors['F9_KEY'].link(game.controllers['Switch_Camera'])
    add_sensor(type='KEYBOARD', name='F12_KEY')
    game.sensors[-1].key = "F12"
    game.sensors[-1].name = "F12_KEY"
    game.sensors['F12_KEY'].link(game.controllers['Fast_quit'])
    add_sensor(type='ALWAYS', name='Constant')
    game.sensors[-1].name = "Constant"
    game.sensors[-1].use_pulse_true_level = True
    game.sensors['Constant'].link(game.controllers['Admin'])
    add_sensor(type='KEYBOARD', name='F11_KEY')
    game.sensors[-1].key = "F11"
    game.sensors[-1].name = "F11_KEY"
    game.sensors['F11_KEY'].link(game.controllers['Restart'])
    # MESH : Screen
    select_only(bpy.data.objects['Screen'])
    bpy.context.object.game.physics_type = 'NO_COLLISION'
    bpy.context.object.hide_render = True
    properties(bpy.context.object, DisplayCamera="CameraMain", Visible=False)
    game = bpy.context.object.game
    add_actuator(type='PROPERTY', name='Toggle')
    game.actuators[-1].mode = "TOGGLE"
    game.actuators[-1].name = "Toggle"
    game.actuators[-1].object_property = ""
    game.actuators[-1].property = "Visible"
    game.actuators[-1].value = ""
    add_actuator(type='VISIBILITY', name='Show')
    game.actuators[-1].apply_to_children = True
    game.actuators[-1].name = "Show"
    game.actuators[-1].use_visible = True
    add_actuator(type='VISIBILITY', name='Hide')
    game.actuators[-1].apply_to_children = True
    game.actuators[-1].use_visible = False
    game.actuators[-1].name = "Hide"
    add_controller(type='LOGIC_AND', name='And')
    game.controllers[-1].name = "And"
    game.controllers['And'].link(actuator=game.actuators['Toggle'])
    add_controller(type='LOGIC_AND', name='And1')
    game.controllers[-1].name = "And1"
    game.controllers['And1'].link(actuator=game.actuators['Show'])
    add_controller(type='LOGIC_NAND', name='Nand')
    game.controllers[-1].name = "Nand"
    game.controllers['Nand'].link(actuator=game.actuators['Hide'])
    add_sensor(type='KEYBOARD', name='V_KEY')
    game.sensors[-1].key = "V"
    game.sensors[-1].name = "V_KEY"
    game.sensors['V_KEY'].link(game.controllers['And'])
    add_sensor(type='PROPERTY', name='Visible')
    game.sensors[-1].name = "Visible"
    game.sensors[-1].property = "Visible"
    game.sensors[-1].value = "True"
    game.sensors[-1].value_min = "True"
    game.sensors['Visible'].link(game.controllers['And1'])
    game.sensors['Visible'].link(game.controllers['Nand'])
    # MESH : Screen_frame
    select_only(bpy.data.objects['Screen_frame'])
    bpy.context.object.game.physics_type = 'NO_COLLISION'
    bpy.context.object.hide_render = True
    game = bpy.context.object.game
    # MESH : Title_text
    select_only(bpy.data.objects['Title_text'])
    bpy.context.object.game.physics_type = 'NO_COLLISION'
    bpy.context.object.hide_render = True
    properties(bpy.context.object, Text="MORSE keyboard  shortcuts")
    game = bpy.context.object.game
    bpy.data.objects['Compass'].parent = bpy.data.objects['CameraFP']
    bpy.data.objects['HUD_plane'].parent = bpy.data.objects['CameraFP']
    bpy.data.objects['Screen'].parent = bpy.data.objects['CameraFP']
    bpy.data.objects['Keys_text'].parent = bpy.data.objects['HUD_plane']
    bpy.data.objects['Title_text'].parent = bpy.data.objects['HUD_plane']
    bpy.data.objects['CameraID_text'].parent = bpy.data.objects['Screen']
    bpy.data.objects['Screen_frame'].parent = bpy.data.objects['Screen']
    # text : setup_path.py
    new_text()
    text = get_last_text()
    text.name = 'setup_path.py'
    text_str = """import sys, os

# Check that MORSE libraries are correctly installed
# Searches the PYTHONPATH variable for the required directories: $MORSE_ROOT and $MORSE_ROOT/morse/blender

def test(contr):
    print ("==============================")
    print ("Welcome to the MORSE simulator")
    print ("==============================")
    blender_dir = 'morse/blender'
    for dir in sys.path:
        if os.path.exists(os.path.join(dir, "morse/blender/main.py")):
            sys.path.append(os.path.join(dir, "morse/blender"))
            sys.path.append(os.path.join(dir, "morse/blender/human_interaction"))
            import imp
            import morse.core.blenderapi
            imp.reload(morse.core.blenderapi)
            return
    print ("MORSE ERROR: could not find the MORSE libraries. Please run 'morse check' from the command line to solve this issue.")
    print ("Exiting the simulation!")
    quitActuator = contr.actuators['Quit_sim']
    contr.activate(quitActuator)
"""
    text.write(text_str)
    text.use_module = True

# test with: blender -P {file}
if __name__ == '__main__':
    init_morse()
