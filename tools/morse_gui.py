import os
import bpy
from morse.builder.morsebuilder import *

data = {}

def init():
    path = MORSE_COMPONENTS
    for category in os.listdir(path):
        pathc = os.path.join(path, category)
        if os.path.isdir(pathc):
            data[category] = []
            for blend in os.listdir(pathc):
                pathb = os.path.join(pathc, blend)
                if os.path.isfile(pathb) & blend.endswith('.blend'):
                    data[category].append(blend[:-6])
                # go to 2nd level (for scenes)
                elif os.path.isdir(pathb):
                    pathb = os.path.join(pathc, blend)
                    if os.path.isfile(pathb) and blend.endswith('.blend'):
                        data[category].append(blend[:-6])
    #print(data)
    for comp in data:
        if len(data[comp]) > 0:
            init_prop(comp, data[comp])

def init_prop(comp, complist):
    objects = []
    for index, name in enumerate(complist):
        objects.append((str(index), name, str(index)))

    enum = bpy.props.EnumProperty(name=comp, description="Choose %s"%comp, items=objects, default='0')
    setattr(bpy.types.Scene, 'enum_%s'%comp, enum)

class MorsePanel(bpy.types.Panel):
    bl_label = "MORSE Panel"
    bl_idname = "OBJECT_PT_MORSE"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "scene"

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        
        for comp in data:
            if len(data[comp]) > 0:
                #row = layout.row()
                #row.label(text="%s: "%comp)
                row = layout.row()
                row.prop(scene, 'enum_%s'%comp)
                row = layout.row()
                row.operator('bpt.add', text="Add MORSE %s"%comp)
                #row.operator('bpt.add_%s'%comp)

class MorseOperator(bpy.types.Operator):
    bl_idname = "bpt.add"
    bl_label = "Add MORSE Component"

    def execute(self, context):
        for i in dir(self):
            try:
                print(i)
                print(getattr(self,i))
            except Exception:
                pass
        #print(self.properties)
        #print(self.name)
        #print(dir(context))
        """
        # TEST
        cat = "robots"
        select = getattr(bpy.context.scene, 'enum_%s'%cat)
        enum = getattr(bpy.types.Scene, 'enum_%s'%cat)
        comp = enum[1]['items'][int(select)][1]
        print(comp)
        Component(cat, comp)
        """
        return{"FINISHED"}

"""    def invoke(self, context, event):
        return self.execute(context)"""

def register():
    init()
    bpy.utils.register_class(MorseOperator)
    bpy.utils.register_class(MorsePanel)

def unregister():
    bpy.utils.unregister_class(MorseOperator)
    bpy.utils.unregister_class(MorsePanel)

if __name__ == "__main__":
    register()

