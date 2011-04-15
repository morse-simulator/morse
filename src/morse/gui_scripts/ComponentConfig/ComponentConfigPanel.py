# ##### BEGIN GPL LICENSE BLOCK #####
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software Foundation,
#  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
#  Written by Gilberto Echeverria at LAAS/CNRS, Toulouse, France
#
# ##### END GPL LICENSE BLOCK #####

# #####
#
# VERSION INFORMATION
# - 23 / 03 / 2011  Start work on the script for middleware-component configuration
# - 14 / 04 / 2011  Switch to Blender 2.57. Correct broken code
#
# TODO:
# - Everything
#
# #####

import bpy
import logging

##
## Before starting setup the logger
##

# create logger
logger = logging.getLogger("ComponentConfigPanel.py")
#logger.setLevel(logging.WARNING)
logger.setLevel(logging.DEBUG)
# create console handler and set level to debug
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
# create formatter
formatter = logging.Formatter("%(levelname)s - %(message)s")
#formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
# add formatter to ch
ch.setFormatter(formatter)
# add ch to logger
logger.addHandler(ch)

## EXAMPLE "application" code
#logger.debug("debug message")
#logger.info("info message")
#logger.warn("warn message")
#logger.error("error message")
#logger.critical("critical message")

print ("\n#### IT BEGINS HERE ####")



class ConfiguredComponent(bpy.types.PropertyGroup):
    """ Custom class to hold the information of a component binding
        
    Will store the name of the component, and the middleware it is bound to
    """
    name = bpy.props.StringProperty(name="name", default="")
    index = bpy.props.IntProperty(name="index", default=0)
    show_expanded = bpy.props.BoolProperty(name='show_expanded', description='View component bindings', default=True)
    component_list = bpy.props.EnumProperty(name='component_list', description='Choose a component', items=[])
    pass

bpy.utils.register_class(ConfiguredComponent)
#bpy.types.register(ConfiguredComponent)


# Define some global variables
# Flag to use or not middlewares
bpy.types.Scene.use_middlewares = bpy.props.BoolProperty(name='use_middlewares', description='Expose MORSE data using middlewares', default=False)
# Counter for the number of bound components
bpy.types.Scene.binding_counter = bpy.props.IntProperty(min = -1, default = -1)

# Create the list of bindings of type 'ConfiguredComponent'
bpy.types.Scene.bound_components = bpy.props.CollectionProperty(type=ConfiguredComponent)



#def updateSceneComponents():
def updateSceneComponents(co_co):
    """ Build a list of the active components in the scene """
    scene_component_list = []
    test_scene_component_list = []
    index = 0

    # Iterate over all objects in the scene, and identify those
    #  tagged as robots or components
    for object in bpy.data.objects:
        try:
            object.game.properties['Component_Tag']
            logger.debug("Object '%s' goes into the list" % object)
            test_scene_component_list.append(object.name)
            scene_component_list.append((str(index), object.name, object.name))
            index = index + 1
        except KeyError as detail:
            pass

    logger.debug("component_list = '{0}'".format(scene_component_list))

    # Create an enum object, which will be used later
    component_enum = bpy.props.EnumProperty(name='component_enum', description='Choose a component', items=scene_component_list, default='0')

    # create an EnumProperty which can be used by the dropdown box
    #  to display the different scenes
    co_co.component_list = test_scene_component_list
    bpy.types.Scene.morse_components = component_enum
    #bpy.types.Scene.morse_components = bpy.props.CollectionProperty(type=bpy.props.StringProperty, items=scene_component_list)


class ROBOTICS_PT_bind_middlewares(bpy.types.Panel):
    ''' Pannel to configure the use of middlewares per component '''
    bl_space_type = 'PROPERTIES' # Window type where the panel will be shown
    bl_region_type = 'WINDOW'
    bl_context = 'scene' # Where to show panel in space_type
    bl_label = 'Morse Component Configuration'
    #bl_options = {'DEFAULT_CLOSED'}

    def draw_header(self, context):
        layout = self.layout
        scene = context.scene
        layout.prop(scene, "use_middlewares", text="")
        lbl = layout.label('Robotics', icon='PLUGIN')
        lbl = layout.label(icon='SNAP_ON')

    @classmethod
    def poll(cls, context):
        scene = context.scene
        # bg = context.space_data.bind_middlewares
        return (scene)

    def draw(self, context):
        layout = self.layout

        scene = context.scene

        col = layout.column()

        # HERE ONLY FOR TESTING
        #updateSceneComponents()
        col.prop(data=context.scene, property='morse_components', text='MORSE component')
        col.operator("robotics.mw_add", text="Add middleware binding")

        for index, co_co in enumerate(scene.bound_components):
            layout.active = scene.use_middlewares
            box = layout.box()
            row = box.row(align=True)
            row.prop(co_co, "show_expanded", text="", emboss=False)

            #lbl = row.label('Component:')
            # draw dropdown box on panel
            row.prop(data=co_co, property='component_list', text='MORSE component')
            #row.prop_search(data=co_co, property="name", search_data=context.scene, search_property="morse_components", icon='OBJECT_DATA')
            #row.label(text=component.name)
            row.operator("robotics.mw_remove", text="", emboss=False, icon='X').index = index

            if co_co.show_expanded:
                info_text = "{0}: {1}".format(co_co.index, co_co.name)
                row = box.row(align=True)
                row.label(text=info_text)
                row.operator("robotics.order_up", icon='TRIA_UP')
                row.operator("robotics.order_down", icon='TRIA_DOWN')

        col = layout.column()
        col.operator("robotics.write_config_file")


class ROBOTICS_OT_refresh_component_list(bpy.types.Operator):
    bl_idname = 'robotics.refresh_component_list'
    bl_label = "Refresh component list" # button label
    bl_description = "Update the list of components in the scene" # Tooltip
    __doc__ = "Update component list"

    def execute(self, context):
        logger.debug ("Repopulating the component list")
        return {'FINISHED'}


class ROBOTICS_OT_order_up(bpy.types.Operator):
    bl_idname = "robotics.order_up"
    bl_label = ""
    __doc__ = "Order up"

    def execute(self, context):
        logger.debug ("Going UP")
        #bpy.ops.view3d.view_orbit(type='ORBITUP')
        return {'FINISHED'}

class ROBOTICS_OT_order_down(bpy.types.Operator):
    bl_idname = "robotics.order_down"
    bl_label = ""
    __doc__ = "Order down"

    def execute(self, context):
        logger.debug ("Going DOWN")
        #bpy.ops.view3d.view_orbit(type='ORBITUP')
        return {'FINISHED'}


class ROBOTICS_OT_mw_add(bpy.types.Operator):
    bl_idname = "robotics.mw_add"
    bl_label = ""
    __doc__ = "Add middleware binding"

    def execute(self, context):
        logger.debug ("Adding new component binding")

        # Increment the counter
        context.scene.binding_counter = context.scene.binding_counter + 1

        # Create a new instance and fill it
        collection = context.scene.bound_components
        collection.add()
        collection[-1].index = context.scene.binding_counter
        collection[-1].name = 'probando'

        # Read the objects in the scene and create a drop down list
        #updateSceneComponents()
        updateSceneComponents(collection[-1])

        return {'FINISHED'}


class ROBOTICS_OT_mw_remove(bpy.types.Operator):
    bl_idname = "robotics.mw_remove"
    bl_label = ""
    __doc__ = "Remove middleware binding"

    # Define an integer property that can be accessed when calling this operator
    index = bpy.props.IntProperty(default=0)

    def execute(self, context):
        collection = context.scene.bound_components
        collection.remove(self.index)
        logger.debug ("Removing middleware binding (index %d)" % self.index)
        return {'FINISHED'}


class ROBOTICS_OT_write_config_file(bpy.types.Operator):
    bl_idname = "robotics.write_config_file"
    bl_label = "Write config file"
    bl_context = 'scene' # Where to show panel in space_type
    __doc__ = "Write config file"

    def execute(self, context):
        print ("THIS IS WHERE THE CONFIG FILE MUST BE CREATED")
        logger.debug ("Writing config file")
        return {'FINISHED'}

bpy.utils.register_class(ROBOTICS_PT_bind_middlewares)
bpy.utils.register_class(ROBOTICS_OT_refresh_component_list)
bpy.utils.register_class(ROBOTICS_OT_order_up)
bpy.utils.register_class(ROBOTICS_OT_order_down)
bpy.utils.register_class(ROBOTICS_OT_mw_add)
bpy.utils.register_class(ROBOTICS_OT_mw_remove)
bpy.utils.register_class(ROBOTICS_OT_write_config_file)
