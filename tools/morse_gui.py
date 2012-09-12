bl_info = {
    "name": "MORSE GUI",
    "author": "Pierrick Koch",
    "version": (1, 0, 0),
    "blender": (2, 5, 9),
    "api": 36147,
    "location": "Properties>Scene",
    "category": "Import-Export",
    "description": "Graphical User Interface for MORSE",
    "warning": "",
    "wiki_url": "",
    "tracker_url": "https://softs.laas.fr/bugzilla/"
}

import os
import bpy
from morse.builder import *

"""
MORSE GUI to import components

To test this module you can open this file inside a Text panel in Blender,
then run the script.
This will generate a GUI in the Properties View in the Scene tab.
"""

data = {}

def init():
    # initialize the component list, scan component directory
    path = MORSE_COMPONENTS
    for category in os.listdir(path):
        pathc = os.path.join(path, category)
        if os.path.isdir(pathc):
            data[category] = list_components(pathc)
            if len(data[category]) > 0:
                init_prop(category, data[category])

def list_components(path, subdir=''):
    components = []
    for blend in os.listdir(path):
        pathb = os.path.join(path, blend)
        if os.path.isfile(pathb) and blend.endswith('.blend'):
            components.append(subdir + blend[:-6])
        # go to 2nd level (for scenes)
        elif os.path.isdir(pathb):
            components.extend(list_components(pathb, subdir='%s%s/'%(subdir, blend)))
    return components

def init_prop(category, components):
    objects = []
    for index, name in enumerate(components):
        objects.append((str(index), name, category))

    enum = bpy.props.EnumProperty(name=category, description="Choose %s"%category, items=objects, default='0')
    setattr(bpy.types.Scene, 'enum_%s'%category, enum)

class MorsePanel(bpy.types.Panel):
    bl_label = "MORSE Panel"
    bl_idname = "OBJECT_PT_MORSE"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "scene"

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        
        for category in data:
            if len(data[category]) > 0:
                row = layout.row()
                row.prop(scene, 'enum_%s'%category)
                row = layout.row()
                btn = row.operator('bpt.add', text="Add MORSE %s"%category)
                btn.category = category

class MorseOperator(bpy.types.Operator):
    bl_idname = "bpt.add"
    bl_label = "Add MORSE Component"
    
    category = bpy.props.StringProperty()

    def execute(self, context):
        # get the index of the selected element
        select = getattr(bpy.context.scene, 'enum_%s'%self.category)
        # get the enum list of the current category
        enum = getattr(bpy.types.Scene, 'enum_%s'%self.category)
        # get the component name
        component = enum[1]['items'][int(select)][1]
        # import the MORSE component
        Component(self.category, component)

        return{"FINISHED"}

def register():
    init() # initialize the component list, scan component directory
    bpy.utils.register_class(MorseOperator)
    bpy.utils.register_class(MorsePanel)

def unregister():
    bpy.utils.unregister_class(MorseOperator)
    bpy.utils.unregister_class(MorsePanel)

if __name__ == "__main__":
    register()

