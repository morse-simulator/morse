bl_info = {
    "name": "Morse Utils",
    "description": "Utils for Morse Simulator. Generates a basic setup for \
     interactable Objects, Doors and Drawers, so the human can use them",
    "author": "Sebastian Schmidt",
    "version": (0,4,99),
    "blender": (2, 5, 0),
    "api": 31236,
    "location": "Logic",
    "warning": '',
    "wiki_url": "",
    "tracker_url": "",
    "category": "Morse"}


import bpy
from bpy.props import *

class MorseObjectDialog(bpy.types.Operator):
    '''
    Operator class that assigns a "Object" and a "Description" Property,
    makes it a Rigid Body with Collision Bounds and Mass
    '''
    bl_idname = "object.morse_object_dialog"
    bl_label = "Morse Object"

    object = BoolProperty(name = "Object")
    graspable = BoolProperty(name = "Graspable")
    label = StringProperty(name = "Label")
    description = StringProperty(name="Description")
    typeProp = StringProperty(name = "Type")
    mass = FloatProperty(name = "Mass")
    bounds = EnumProperty(name="Collision Bounds",
        items = [('one', 'Box', 'Box'), 
                 ('two', 'Sphere', 'Sphere'),
                 ('three', 'Capsule', 'Capsule'), 
                 ('four', 'Cylinder', 'Cylinder'),
                 ('five', 'Cone', 'Cone'),
                 ('six', 'Convex Hull', 'Convex Hull'),
                 ('seven', 'Triangle Mesh', 'Triangle Mesh')])
    
    items = {'one':'BOX', 'two':'SPHERE', 'three':'CAPSULE', 'four':'CYLINDER',
             'five':'CONE', 'six':'CONVEX_HULL', 'seven':'TRIANGLE_MESH'}
    inv_items = {'BOX': 'one', 'CYLINDER': 'four', 'TRIANGLE_MESH': 'seven',
                 'SPHERE': 'two', 'CAPSULE': 'three', 'CONE': 'five',
                 'CONVEX_HULL': 'six'}

    
    def assignProp(self, context, name, value):
        prop = context.object.game.properties[name]
        prop.value = value
    
    def execute(self, context):
        obj = bpy.context.active_object
        
        # add game properties if needed
        if not 'Object' in obj.game.properties:
            bpy.ops.object.game_property_new()
            prop = context.object.game.properties[-1]
            prop.name = 'Object'
            prop.type = 'BOOL'
        
        if not 'Description' in obj.game.properties:
            bpy.ops.object.game_property_new()
            prop = context.object.game.properties[-1]
            prop.name = 'Description'
            prop.type = 'STRING'

        if not 'Label' in obj.game.properties:
            bpy.ops.object.game_property_new()
            prop = context.object.game.properties[-1]
            prop.name = 'Label'
            prop.type = 'STRING'

        if not 'Type' in obj.game.properties:
            bpy.ops.object.game_property_new()
            prop = context.object.game.properties[-1]
            prop.name = 'Type'
            prop.type = 'STRING'

        if not 'Graspable' in obj.game.properties:
            bpy.ops.object.game_property_new()
            prop = context.object.game.properties[-1]
            prop.name = 'Graspable'
            prop.type = 'BOOL'
    
        self.assignProp(context, 'Object', self.object)
        self.assignProp(context, 'Description', self.description)
        self.assignProp(context, 'Label', self.label)
        self.assignProp(context, 'Type', self.typeProp)
        self.assignProp(context, 'Graspable', self.graspable)
        
        if not 'Collision' in obj.game.sensors:
            bpy.ops.logic.sensor_add(type = 'NEAR')
            sens = context.object.game.sensors[-1]
            sens.name = 'Collision'
            sens.distance = 0.05
            sens.reset_distance = 0.075

            bpy.ops.logic.controller_add()
            contr = context.object.game.controllers[-1]

            contr.link(sensor = sens)

        
        # make the object a rigid body
        obj.game.physics_type = 'RIGID_BODY'
        obj.game.use_collision_bounds = True
        obj.game.collision_bounds_type = self.items[self.bounds]
        obj.game.mass = self.mass
        
        return {'FINISHED'}
 
    def invoke(self, context, event):
        obj = context.object
        # check not to overwrite anything already set
        if 'Object' in obj.game.properties:
            self.object = obj.game.properties['Object'].value
        else:
            self.object = True
        if 'Label' in obj.game.properties:
            self.label = obj.game.properties['Label'].value
        else:
            self.label = context.object.name
        if 'Description' in obj.game.properties:
            self.description = obj.game.properties['Description'].value
        else:
            self.description = context.object.name
        if 'Graspable' in obj.game.properties:
            self.graspable = obj.game.properties['Graspable'].value
        else:
            self.graspable = True
        if 'Type' in obj.game.properties:
            self.typeProp = obj.game.properties['Type'].value


        
        self.mass = obj.game.mass
        self.bounds = self.inv_items[obj.game.collision_bounds_type]
        return context.window_manager.invoke_props_dialog(self)

 
class MorseDrawerDialog(bpy.types.Operator):
    '''
    Operator class that assigns a "Drawer", a "Description" and a "Open"
    Property, and the logic needed for Animation
    '''
    bl_idname = "object.morse_drawer_dialog"
    bl_label = "Morse Drawer"
 
    drawer = StringProperty(name = "Drawer")
    description = StringProperty(name="Description")
    end_frame = IntProperty(name = "End Frame")
    open = BoolProperty(name = "Open")
        
    def assignProp(self, context, name, value):
        prop = context.object.game.properties[name]
        prop.value = value
    
    def execute(self, context):
        obj = bpy.context.active_object
        
        # add game properties if needed
        if not 'Drawer' in obj.game.properties:
            bpy.ops.object.game_property_new()
            prop = context.object.game.properties[-1]
            prop.name = 'Drawer'
            prop.type = 'STRING'
        
        if not 'Open' in obj.game.properties:
            bpy.ops.object.game_property_new()
            prop = context.object.game.properties[-1]
            prop.name = 'Open'
            prop.type = 'BOOL'
        
        if not 'Description' in obj.game.properties:
            bpy.ops.object.game_property_new()
            prop = context.object.game.properties[-1]
            prop.name = 'Description'
            prop.type = 'STRING'
            
        self.assignProp(context, 'Drawer', self.drawer)
        self.assignProp(context, 'Open', self.open)
        self.assignProp(context, 'Description', self.description)
        
        # make the drawer an actor
        obj.game.use_actor = True
        
        # set the logic to open and close the drawer if needed
        if 'Open' not in context.object.game.sensors:
            bpy.ops.logic.sensor_add(type = 'PROPERTY')
            sens = context.object.game.sensors[-1]
            sens.name = 'Open'
            sens.property = 'Open'
            sens.value = 'True'
        
            bpy.ops.logic.controller_add()
            contr = context.object.game.controllers[-1]
        
            contr.link(sensor=sens)
        
            if bpy.app.version < (2, 60, 0):
                bpy.ops.logic.actuator_add(type = 'FCURVE')
            else:
                bpy.ops.logic.actuator_add(type = 'ACTION')
            act = context.object.game.actuators[-1]
            act.name = 'Open'
        
            contr.link(actuator=act)
        
        
        if 'Close' not in context.object.game.sensors:
            bpy.ops.logic.sensor_add(type = 'PROPERTY')
            sens = context.object.game.sensors[-1]
            sens.name = 'Close'
            sens.property = 'Open'
            sens.value = 'False'
        
            bpy.ops.logic.controller_add()
            contr = context.object.game.controllers[-1]
        
            contr.link(sensor=sens)
            
            if bpy.app.version < (2, 60, 0):
                bpy.ops.logic.actuator_add(type = 'FCURVE')
            else:
                bpy.ops.logic.actuator_add(type = 'ACTION')
            act = context.object.game.actuators[-1]
            act.name = 'Close'
        
            contr.link(actuator=act)
        
        context.object.game.actuators['Open'].frame_end = self.end_frame
        context.object.game.actuators['Close'].frame_start = self.end_frame
        
        return {'FINISHED'}
 
    def invoke(self, context, event):
        obj = context.object
        # check not to overwrite anything already set
        if 'Drawer' in obj.game.properties:
            self.drawer = obj.game.properties['Drawer'].value
        else:
            self.drawer = context.object.name
        if 'Description' in obj.game.properties:
            self.description = obj.game.properties['Description'].value
        else:
            self.description = context.object.name
        if 'Open' in obj.game.properties:
            self.open = obj.game.properties['Open'].value
        else:
            self.open = False
        if 'Open' in obj.game.actuators:
            self.end_frame = obj.game.actuators['Open'].frame_end
        else:
            self.end_frame = 40
        return context.window_manager.invoke_props_dialog(self)


class MorseDoorDialog(bpy.types.Operator):
    '''
    Operator class that assigns a "Door", a "Description" and a "Open" Property
    In Future the may be animated as well, a logic setup is already in this
    script, but commented.
    '''
    bl_idname = "object.morse_door_dialog"
    bl_label = "Morse Door"
 
    door = StringProperty(name = "Door")
    description = StringProperty(name="Description")
    # uncomment next line if using the IPO Setup
    # end_frame = IntProperty(name = "End Frame")
    open = BoolProperty(name = "Open")
    
    def assignProp(self, context, name, value):
        prop = context.object.game.properties[name]
        prop.value = value
    
    def execute(self, context):
        obj = bpy.context.active_object
        
        # add game properties if needed
        if not 'Door' in obj.game.properties:
            bpy.ops.object.game_property_new()
            prop = context.object.game.properties[-1]
            prop.name = 'Door'
            prop.type = 'STRING'
        
        if not 'Open' in obj.game.properties:
            bpy.ops.object.game_property_new()
            prop = context.object.game.properties[-1]
            prop.name = 'Open'
            prop.type = 'BOOL'
        
        if not 'Description' in obj.game.properties:
            bpy.ops.object.game_property_new()
            prop = context.object.game.properties[-1]
            prop.name = 'Description'
            prop.type = 'STRING'
        
        self.assignProp(context, 'Door', self.door)
        self.assignProp(context, 'Open', self.open)
        self.assignProp(context, 'Description', self.description)
        
        # make the drawer an actor
        obj.game.use_actor = True

        # uncomment this part as well for the IPO Setup
        '''
        # set the logic to open and close the door if needed
        if 'Open' not in context.object.game.sensors:
            bpy.ops.logic.sensor_add(type = 'PROPERTY')
            sens = context.object.game.sensors[-1]
            sens.name = 'Open'
            sens.property = 'Open'
            sens.value = 'True'
        
            bpy.ops.logic.controller_add()
            contr = context.object.game.controllers[-1]
        
            contr.link(sensor=sens)
        
            if bpy.app.version < (2, 60, 0):
                bpy.ops.logic.actuator_add(type = 'FCURVE')
            else:
                bpy.ops.logic.actuator_add(type = 'ACTION')
            act = context.object.game.actuators[-1]
            act.name = 'Open'
        
            contr.link(actuator=act)
        
        
        if 'Close' not in context.object.game.sensors:
            bpy.ops.logic.sensor_add(type = 'PROPERTY')
            sens = context.object.game.sensors[-1]
            sens.name = 'Close'
            sens.property = 'Open'
            sens.value = 'False'
        
            bpy.ops.logic.controller_add()
            contr = context.object.game.controllers[-1]
        
            contr.link(sensor=sens)
        
            if bpy.app.version < (2, 60, 0):
                bpy.ops.logic.actuator_add(type = 'FCURVE')
            else:
                bpy.ops.logic.actuator_add(type = 'ACTION')
            act = context.object.game.actuators[-1]
            act.name = 'Close'
        
            contr.link(actuator=act)
        
        context.object.game.actuators['Open'].frame_end = self.end_frame
        context.object.game.actuators['Close'].frame_start = self.end_frame
        '''
        return {'FINISHED'}
 
    def invoke(self, context, event):
        obj = context.object
        # check not to overwrite anything already set
        if 'Door' in obj.game.properties:
            self.door = obj.game.properties['Door'].value
        else:
            self.door = 'right'
        if 'Description' in obj.game.properties:
            self.description = obj.game.properties['Description'].value
        else:
            self.description = context.object.name
        if 'Open' in obj.game.properties:
            self.open = obj.game.properties['Open'].value
        else:
            self.open = False
        # uncomment this as well for the IPO Setup
        '''
        if 'Open' in obj.game.actuators:
            self.end_frame = obj.game.actuators['Open'].frame_end
        else:
            self.end_frame = 40
        '''
        return context.window_manager.invoke_props_dialog(self)

# Panel in tools region
class MorsePanel(bpy.types.Panel):
    '''
    Adds a Panel in the Logic Editor UI
    '''
    bl_label = "Morse Utils"
    bl_space_type = "LOGIC_EDITOR"
    bl_region_type = "UI"
 
    def draw(self, context):
        layout = self.layout
        layout.operator("object.morse_object_dialog")
        layout.operator("object.morse_drawer_dialog")
        layout.operator("object.morse_door_dialog")
 

def register():
    bpy.utils.register_class(MorseObjectDialog)
    bpy.utils.register_class(MorseDoorDialog)
    bpy.utils.register_class(MorseDrawerDialog)
    bpy.utils.register_class(MorsePanel)
def unregister():
    bpy.utils.unregister_class(MorseObjectDialog)
    bpy.utils.unregister_class(MorseDoorDialog)
    bpy.utils.unregister_class(MorseDrawerDialog)
    bpy.utils.unregister_class(MorsePanel)
