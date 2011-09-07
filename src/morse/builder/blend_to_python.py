#--- ### Header
bl_info = {
    "name": "Blender_to_Python",
    "author": "Gilberto Echeverria",
    "version": (1, 0, 0),
    "blender": (2, 5, 9),
    "api": 36147,
    "location": "View3D>Edit Mode>Specials (W-key)",
    "category": "Import-Export",
    "description": "Scan a Blender scene for robots and save their configurationas a text file readable by the 'morsebuilder' script",
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
Morse API

To test this module you can c/p the following code in Blender Python console::

import sys
sys.path.append("/usr/local/lib/python3.1/dist-packages")
from morse.builder.morsebuilder import *
atrv=Robot("atrv")

The string passed to the differents Components Classes must be an existing 
.blend file-name, ie. for ``Robot("atrv")`` the file ``atrv.blend`` must exists 
in the folder ``MORSE_COMPONENTS/robots/``.
"""

morse_types = {
    "robots": "Robot",
    "sensors": "Sensor",
    "actuators": "Actuators",
    "middleware": "Middleware",
}

def scan_scene (out_file):
    """ Read all the MORSE components from a Blender file

    Create lists of robots and components to save them as a text file
    """

    out_file.write("from morse.builder.morsebuilder import *\n\n")

    for obj in bpy.data.objects:
        try:
            component_path = obj.game.properties['Path'].value
        # Exit if the object is not a MORSE component
        except KeyError as detail:
            continue

        path_elements = component_path.split('/')
        component_type = path_elements[-2]
        component_name = path_elements[-1]

        builder_type = morse_types[component_type]

        obj_name = re.sub('\.', '_', obj.name)
        # Create the object instance
        out_file.write("%s = %s('%s')\n" % (obj_name, builder_type, component_name))
        # Set its position
        position_string = ''
        component_position = obj.location
        if component_position[0] != 0:
            position_string += 'x=' + str(component_position[0])
        if component_position[1] != 0:
            if position_string != '':
                position_string += ', '
            position_string += 'y=' + str(component_position[1])
        if component_position[2] != 0:
            if position_string != '':
                position_string += ', '
            position_string += 'z=' + str(component_position[2])
        # Register a translation only if necessary
        if position_string != '':
            out_file.write("%s.translate(%s)\n" % (obj_name, position_string))

        # Assign component to the parent
        if component_type == 'sensors' or component_type == 'actuators':
            parent_name = re.sub('\.', '_', obj.parent.name)
            out_file.write("%s.append(%s)\n" % (parent_name, obj_name))

            # Store the properties of the component
            for key,value in obj.game.properties.items():
                if key not in ['Robot_Tag', 'Component_Tag', 'Middleware_Tag', 'Modifier_Tag', 'Class', 'Path']:
                    if value.value != '':
                        out_file.write("%s.properties(%s = %s)\n" % (obj_name, key, value.value))

        out_file.write("\n")

def scan_config(file_out):
    """ Parse the contents of 'component_config.py'
    
    Produce a configuration file that 'morsebuilder' can use to
    configure the robot/middleware bindings in a scene. 
    """
    import component_config
    for key,value in component_config.component_mw.items():
        component = re.sub('\.', '_', key)
        mw = value[0]
        file_out.write("%s.configure_mw(%s, %s)\n" % (mw, component, value))

    for key,value in component_config.component_service.items():
        component = re.sub('\.', '_', key)
        mw = re.search('(\w+)_request_manager', value[0])
        file_out.write("%s.configure_service('%s')\n" % (component, mw.group(1)))


def main():
    file_out = open("backup.py", "w")
    scan_scene(file_out)
    scan_config(file_out)
    file_out.write("\nConfiguration.write()")
    file_out.close()


#--- ### Operator
class BlenderToPython(bpy.types.Operator):
    ''' Convert a MORSE scene configuration to a python script '''
    bl_idname = "export.blender_to_python"
    bl_label = "Blender to Python"
    bl_description = "Convert a MORSE scene configuration to a python script"

    def execute(self,context):
        main()
        return ('FINISHED')


#--- ### Register
def register():
    register_module(__name__)
def unregister():
    unregister_module(__name__)

#--- ### Main code
if __name__ == '__main__':
    main()
