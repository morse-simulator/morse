#--- ### Header
bl_info = {
    "name": "MORSE export scene as Python API (.py)",
    "author": "Gilberto Echeverria",
    "version": (1, 0, 0),
    "blender": (2, 5, 9),
    "api": 36147,
    "location": "File>Import-Export",
    "category": "MORSE",
    #"category": "Import-Export",
    "description": "Save a MORSE scene as a Python description",
    "warning": "",
    "wiki_url": "",
    "tracker_url": "https://softs.laas.fr/bugzilla/"
}

import os
import bpy
import json
import re
from morse.builder.data import *

"""
Morse API to save scene files

To test this module you can open this file inside a Text panel in Blender,
then run the script.
This will generate a python file in the same directory where Blender was first executed.
"""

morse_types = {
    "robots": "Robot",
    "sensors": "Sensor",
    "actuators": "Actuator",
    "middleware": "Middleware",
    "modifiers": "Modifier",
}

def save_translation(obj, obj_name):
    # Set its position
    position_string = ''
    text_buffer = ''
    component_position = obj.location
    if component_position[0] != 0:
        position_string += 'x=%.4f' % component_position[0]
    if component_position[1] != 0:
        if position_string != '':
            position_string += ', '
        position_string += 'y=%.4f' % component_position[1]
    if component_position[2] != 0:
        if position_string != '':
            position_string += ', '
        position_string += 'z=%.4f' % component_position[2]
    # Register a translation only if necessary
    if position_string != '':
        text_buffer += "%s.translate(%s)\n" % (obj_name, position_string)

    return (text_buffer)


def save_rotation(obj, obj_name):
    # Set its rotation
    rotation_string = ''
    text_buffer = ''
    component_rotation = obj.rotation_euler
    if component_rotation[0] != 0:
        rotation_string += 'x=%.4f' % component_rotation[0]
    if component_rotation[1] != 0:
        if rotation_string != '':
            rotation_string += ', '
        rotation_string += 'y=%.4f' % component_rotation[1]
    if component_rotation[2] != 0:
        if rotation_string != '':
            rotation_string += ', '
        rotation_string += 'z=%.4f' % component_rotation[2]
    # Register a translation only if necessary
    if rotation_string != '':
        text_buffer += "%s.rotate(%s)\n" % (obj_name, rotation_string)

    return (text_buffer)


def save_properties(obj, obj_name):
    text_buffer = ''
    # Store the properties of the component
    for key,prop in obj.game.properties.items():
        if key not in ['Robot_Tag', 'Component_Tag', 'Middleware_Tag', 'Modifier_Tag', 'Class', 'Path']:
            if prop.value != '':
                if prop.type == 'STRING':
                    text_buffer += "%s.properties(%s = '%s')\n" % (obj_name, key, prop.value)
                elif prop.type == 'FLOAT' or prop.type == 'TIMER':
                    text_buffer += "%s.properties(%s = %.4f)\n" % (obj_name, key, prop.value)
                else:
                    text_buffer += "%s.properties(%s = %s)\n" % (obj_name, key, prop.value)

    return (text_buffer)


def scan_scene (file_out):
    """ Read all the MORSE components from a Blender file

    Create lists of robots and components to save them as a text file
    """
    file_out.write("from morse.builder import *\n\n")

    robot_text = ''
    component_text = ''

    for obj in bpy.data.objects:
        try:
            component_path = obj.game.properties['Path'].value
        # Exit if the object is not a MORSE component
        except KeyError as detail:
            continue

        # Ignore middleware and modifier empties.
        # These will be added dinamically by the builder
        if 'middleware' in component_path or 'modifiers' in component_path:
            continue

        # Read what type of component this is,
        #  from the source of its python file
        path_elements = component_path.split('/')
        component_type = path_elements[-2]
        component_name = path_elements[-1]

        builder_type = morse_types[component_type]

        # Swap dots for underscores in object names
        obj_name = re.sub('\.', '_', obj.name)
        # Create the object instance
        if component_type == 'robots':
            robot_text += "%s = %s('%s')\n" % (obj_name, builder_type, component_name)
            robot_text += save_translation(obj, obj_name)
            robot_text += save_rotation(obj, obj_name)
            robot_text += save_properties(obj, obj_name)
            robot_text += "\n"

        # Assign component to the parent
        if component_type == 'sensors' or component_type == 'actuators':
            component_text += "%s = %s('%s')\n" % (obj_name, builder_type, component_name)
            component_text += save_translation(obj, obj_name)
            component_text += save_rotation(obj, obj_name)
            parent_name = re.sub('\.', '_', obj.parent.name)
            component_text += "%s.append(%s)\n" % (parent_name, obj_name)
            component_text += save_properties(obj, obj_name)
            component_text += "\n"

    # Write the buffers to the text file
    file_out.write("# Robots\n")
    file_out.write(robot_text)
    file_out.write("# Components\n")
    file_out.write(component_text)


def scan_config(file_out):
    """ Parse the contents of 'component_config.py'
    
    Produce a configuration file that 'morsebuilder' can use to
    configure the robot/middleware bindings in a scene. 
    """
    import component_config
    file_out.write("# Scene configuration\n")
    for key,value in component_config.component_mw.items():
        component = re.sub('\.', '_', key)
        # If the 'value' variable contains only strings, use that string
        #  as the name of the middleware.
        # This is done for backwards compatibility with the previous
        #  syntax that allowed only one middleware per component
        if isinstance (value[0], str):
            mw = value[0]
            mw = mw.lower()
            file_out.write("%s.configure_mw('%s', %s)\n" % (component, mw, value))
        # If using the new syntax that allows more than one middleware
        #  per component
        else:
            for item in value:
                mw = item[0]
                mw = mw.lower()
                file_out.write("%s.configure_mw('%s', %s)\n" % (component, mw, item))

    try:
        component_config.component_service
        file_out.write("\n")
        for key,value in component_config.component_service.items():
            component = re.sub('\.', '_', key)
            mw = re.search('(\w+)_request_manager', value[0])
            file_out.write("%s.configure_service('%s')\n" % (component, mw.group(1)))
    except AttributeError as detail:
        print ("\tNo services configured")

    try:
        component_config.component_modifier
        file_out.write("\n")
        for key,value in component_config.component_modifier.items():
            component = re.sub('\.', '_', key)
            mod = value[0]
            file_out.write("%s.configure_modifier(%s)\n" % (component, mod))
    except AttributeError as detail:
        print ("\tNo modifiers configured")

def get_environment():
    try:
        ssh = bpy.data.objects['Scene_Script_Holder']
        environment_file = ssh.game.properties['environment_file'].value
    except KeyError as detail:
        environment_file = 'indoors-1/indoor-1'
        print ("No environment file specified in 'Scene_Script_Holder'\nUsing '%s' as default" % environment_file)

    return environment_file


def save_scene():
    print ("\nRunning from %s" % bpy.data.filepath)
    filename = bpy.path.display_name_from_filepath(bpy.data.filepath) + ".py"
    file_out = open(filename, "w")
    print ("Saving scene robot configuration to file '%s'" % filename)
    scan_scene(file_out)
    scan_config(file_out)
    env_name = get_environment()
    file_out.write("\nenv = Environment('%s')" % env_name)
    file_out.write("\nenv.create()")
    file_out.close()
    print ("Configuration saved")


#--- ### Operator
class MorseExporter(bpy.types.Operator):
    ''' Convert a MORSE scene configuration to a python script '''
    bl_idname = "export_scene.morse"
    bl_label = "Save MORSE scene"
    bl_description = "Convert a MORSE scene configuration to a python script"

    #--- Blender interface methods
    # Check that this is a MORSE scene
    @classmethod
    def poll(self, context):
        return ('Scene_Script_Holder' in bpy.data.objects)

    def execute(self, context):
        save_scene()
        return {'FINISHED'}


def menu_draw(self, context):
    self.layout.operator_context = 'INVOKE_REGION_WIN'
    self.layout.operator(MorseExporter.bl_idname, "Save MORSE scene (.py)")

#--- ### Register
def register():
    bpy.utils.register_module(__name__)
    bpy.types.INFO_MT_file_export.prepend(menu_draw)
def unregister():
    bpy.types.INFO_MT_file_export.remove(menu_draw)
    bpy.utils.unregister_module(__name__)

#--- ### Main code
if __name__ == '__main__':
    register()
    #save_scene()
