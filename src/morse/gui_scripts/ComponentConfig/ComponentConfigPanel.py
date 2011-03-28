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
logger.setLevel(logging.WARNING)
#logger.setLevel(logging.DEBUG)
# create console handler and set level to debug
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
# create formatter
formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
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


bpy.types.Scene.use_middlewares = bpy.props.BoolProperty(name='use_middlewares', description='Expose MORSE data using middlewares', default=False)
bpy.types.Scene.bound_components = []



#class ConfiguredComponent(bpy.types.AnyType):
##class ConfiguredComponent(bpy.types.PropertyGroup):
#    """ Custom class to hold the information of a component """
#    name = bpy.props.StringProperty(name="name", default="")
#    index = bpy.props.IntProperty(name="index", default=0)
#    show_expanded = bpy.props.BoolProperty(name='show_expanded', description='View component bindings', default=True)
#
#    #def __init__(self, index, name):
#    #    self.name = name
#    #    self.index = index
#
#    def __str__(self):
#        return ("This is me: %d | %s | %s" % (self.index, self.name, self.show_expanded))
#
#    def __repr__(self):
#        return ("This is my representation: %d | %s | %s" % (self.index, self.name, self.show_expanded))
#
##bpy.utils.register_class(ConfiguredComponent)

class ConfiguredComponent(object):
    name = ""
    index = 0
    object = None
    mw_list = []
    show_expanded = False

    def __init__(self, index, name):
        self.name = name
        self.index = index

    def __str__(self):
        return ("This is me: %d | %s | %s" % (self.index, self.name, self.show_expanded))

    def __repr__(self):
        return ("This is my representation: %d | %s | %s" % (self.index, self.name, self.show_expanded))


def updateSceneComponents():
    component_list = []
    index = 0

    # Iterate over all objects in the scene, and identify those
    #  tagged as robots or components
    for object in bpy.data.objects:
        try:
            object.game.properties['Component_Tag']
            logger.debug("Object '%s' goes into the list" % object)
            component_list.append((str(index), object.name, object.name))
            #component_list.append((object, object.name, object.name))
            index = index + 1
        except KeyError as detail:
            pass

    logger.debug("component_list = '{0}'".format(component_list))

    # create an EnumProperty which can be used by the dropdown box
    #  to display the different scenes
    bpy.types.Scene.components = bpy.props.EnumProperty(name='components', description='Choose a component', items=component_list, default='0')



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
        col.operator("robotics.mw_add", text="Add middleware binding")

        updateSceneComponents()

        for index, component in enumerate(scene.bound_components):
            print ("WHAT?? {0}".format(component))
            layout.active = scene.use_middlewares
            box = layout.box()
            row = box.row(align=True)
            #row.prop(component, "show_expanded", text="", emboss=False)

            #lbl = row.label('Component:')
            # draw dropdown box on panel
            row.prop(data=context.scene, property='components', text='MORSE component')
            #row.label(text=component.name)
            row.operator("robotics.mw_remove", text="", emboss=False, icon='X')
            #row.operator("robotics.mw_remove", text="", emboss=False, icon='X').index = index

            #box.prop(component, "view_axis", text="Axis")

            if component.show_expanded:
                row = box.row()
                row.label(text="SOMETHING ELSE GOES HERE")
                #row.template_ID(component, "image", open="image.open")
                #if (component.image):
                #    box.template_image(component, "image", component.image_user, compact=True)

                #    box.prop(component, "opacity", slider=True)
                #    if component.view_axis != 'CAMERA':
                #        box.prop(component, "size")
                #        row = box.row(align=True)
                #        row.prop(component, "offset_x", text="X")
                #        row.prop(component, "offset_y", text="Y")

        col = layout.column()
        col.operator("robotics.write_config_file")


class ROBOTICS_PT_component_config(bpy.types.Panel):
    ''' Pannel to configure the use of middlewares per component '''
    bl_space_type = 'PROPERTIES' # Window type where the panel will be shown
    bl_region_type = 'WINDOW'
    bl_context = 'scene' # Where to show panel in space_type
    bl_label = 'Morse Component Configuration'

    def draw_header(self, context):
        layout = self.layout
        lbl = layout.label('Robotics', icon='PLUGIN')
        lbl = layout.label(icon='SNAP_ON')

    def draw(self, context):
        layout = self.layout

        lbl = layout.label('Component:')

        sep = layout.separator()

        updateSceneComponents()
        # draw dropdown box on panel
        layout.prop(data=context.scene, property='components', text='MORSE components')

        box = layout.box()
        box.label("Header")
        row = box.row()
        row.operator("object.select_all")
        row = box.row(align=True)
        row.operator("robotics.order_up", icon='TRIA_UP')
        row.operator("robotics.order_down", icon='TRIA_DOWN')
        #row.operator("bpy.ops.object.modifier_move_up")


class ROBOTICS_OT_RefreshComponentList(bpy.types.Operator):
    bl_idname = 'ROBOTICS_OT_RefreshComponentList' # robotics.RefreshComponentList (name used to refer to this operator)
    bl_label = 'Refresh component list' # button label
    bl_description = 'Update the list of components in the scene' # Tooltip


class ROBOTICS_OT_order_up(bpy.types.Operator):
    bl_idname = "ROBOTICS_OT_order_up"
    bl_label = ""
    __doc__ = "Order up"

    def execute(self, context):
        logger.debug ("Going UP")
        #bpy.ops.view3d.view_orbit(type='ORBITUP')
        return {'FINISHED'}

class ROBOTICS_OT_order_down(bpy.types.Operator):
    bl_idname = "ROBOTICS_OT_order_down"
    bl_label = ""
    __doc__ = "Order down"

    def execute(self, context):
        logger.debug ("Going DOWN")
        #bpy.ops.view3d.view_orbit(type='ORBITUP')
        return {'FINISHED'}


class ROBOTICS_OT_mw_add(bpy.types.Operator):
    bl_idname = "ROBOTICS_OT_mw_add"
    bl_label = ""
    __doc__ = "Add middleware binding"

    index = 0

    def execute(self, context):
        logger.debug ("Adding new middleware")
        # Read the objects in the scene and create a drop down list
        updateSceneComponents()
        coco = ConfiguredComponent(self.index, 'probando')
        bpy.types.Scene.bound_components.append( coco )
        self.index = self.index + 1
        #bpy.ops.view3d.view_orbit(type='ORBITUP')
        return {'FINISHED'}


class ROBOTICS_OT_mw_remove(bpy.types.Operator):
    bl_idname = "ROBOTICS_OT_mw_remove"
    bl_label = ""
    __doc__ = "Remove middleware binding"

    index = 0

    def execute(self, context):
        print ("AT LEAST I GOT HERE")
        logger.debug ("Removing middleware binding")
        #bpy.ops.view3d.view_orbit(type='ORBITUP')
        return {'FINISHED'}


class ROBOTICS_PT_write_config_file(bpy.types.Operator):
    bl_idname = "ROBOTICS_OT_write_config_file"
    bl_label = "Write config file"
    __doc__ = "Write config file"

    def execute(self, context):
        print ("THIS IS WHERE THE CONFIG FILE MUST BE CREATED")
        logger.debug ("Writing config file")
        return {'FINISHED'}
