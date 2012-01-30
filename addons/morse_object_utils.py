bl_info = {
    "name": "Morse Utils",
    "description": "Utils for Environment Creation for MORSE",
    "author": "Sebastian Schmidt",
    "version": (0,5,0),
    "blender": (2, 61, 0),
    "api": 39307,
    "location": "Logic",
    "warning": '',
    "wiki_url": "",
    "tracker_url": "",
    "category": "Morse"}


import bpy
from bpy.props import *


def update(self, context):
    objects = []
    for index, obj in enumerate(bpy.context.scene.objects):
        objects.append((str(index), obj.name, str(index)))
    return objects
     
class MorseSwitchDialog(bpy.types.Operator):
    '''
    Make Switches usable for the Human
    '''
    bl_idname = "object.morse_switch_dialog"
    bl_label = "Morse Switch"


    objs = {}
    inv_objs = {}
    
    try:
        bpy.app.handlers.scene_update_post
        handler_available = True
        master = EnumProperty(name = "Switch",items = update)
    except AttributeError:
        handler_available = False
        master = StringProperty(name = "Switch")
    on = BoolProperty(name = "On")

    def update_dict(self, context):
        objs = {}
        inv_objs = {}
        for index, obj in enumerate(bpy.context.scene.objects):
            objs[str(index)] = obj.name
            inv_objs[obj.name] = str(index)
        self.objs = objs
        self.inv_objs = inv_objs
        
    
    def assignProp(self, context, name, value):
        prop = context.object.game.properties[name]
        prop.value = value

    def execute(self, context):
        obj = context.active_object

        obj.game.use_actor = True

        if not 'Switch' in obj.game.properties:
            bpy.ops.object.game_property_new()
            prop = context.object.game.properties[-1]
            prop.name = 'Switch'
            prop.type = 'STRING'

        self.assignProp(context, 'Switch', self.objs[self.master] if self.handler_available else self.master)

        
        try:
            master = context.scene.objects[obj.game.properties['Switch'].value]
            context.scene.objects.active = master
            if not 'On' in master.game.properties:
                bpy.ops.object.game_property_new()
                prop = context.object.game.properties[-1]
                prop.name = 'On'
                prop.type = 'BOOL'

            self.assignProp(context, 'On', self.on)
        except KeyError:
            self.report({'INFO'}, "No object with this name - switch will not function in simulation")

        return{'FINISHED'}
            
    def invoke(self, context, event):
        self.update_dict(context)
        obj = context.object

        if 'Switch' in obj.game.properties:
            if self.handler_available:
                self.master = self.inv_objs[obj.game.properties['Switch'].value]
            else:
                self.master = obj.game.properties['Switch'].value

        return context.window_manager.invoke_props_dialog(self)
    
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
        else:
            self.typeProp = "Object"


        
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

    if bpy.app.version >= (2, 60, 0):
        direction = EnumProperty(name = "Generate Action",
                             items = [('one', 'No', 'one'),
                                      ('two', 'X+', 'two'),
                                      ('three', 'X-', 'three'),
                                      ('four', 'Y+', 'four'),
                                      ('five', 'Y-', 'five')])

        vec = {'two':(1.0, 0.0, 0.0), 'three':(-1.0, 0.0, 0.0), 'four':(0.0, 1.0, 0.0), 'five':(0.0, -1.0, 0.0)}
    else:
        direction = 'one'
        # means no action generation
    
    def generate_action(self, endframe, direction):
        # create an action
        action = bpy.data.actions.new(name = bpy.context.object.name + 'Open')
        bpy.context.object.animation_data_create()
        bpy.context.object.animation_data.action = action
        
        # set starting keyframe
        bpy.ops.anim.change_frame(frame = 0)
        bpy.ops.anim.keyframe_insert(type = 'Location')
        
        # set endframe 
        bpy.ops.anim.change_frame(frame = endframe)
        dim = bpy.context.object.dimensions
        translate_tuple = tuple(0.8 * dim[i] * direction[i] for i in range(0,3))    
        bpy.ops.transform.translate(value = translate_tuple)
        bpy.ops.anim.keyframe_insert(type = 'Location')
        
        
        bpy.ops.anim.change_frame(frame = 0)
        
        return action
        
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

        # generate action
        if self.direction != 'one':
            action = self.generate_action(self.end_frame,self.vec[self.direction])
        else:
            action = None
        
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
            if action:
                act.action = action
        
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
            if action:
                act.action = action
        
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
        
        # make the door an actor
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

class MorseLightDialog(bpy.types.Operator):
    '''
    Set up a light that can be switched on and off
    '''
    bl_idname = "object.morse_light_dialog"
    bl_label = "Morse Light"

    On = BoolProperty(name = "On")
    energy = FloatProperty(name = "Energy")

    def assignProp(self, context, name, value):
        prop = context.object.game.properties[name]
        prop.value = value

    def execute(self, context):
        obj = bpy.context.active_object
        
        # add game properties if needed
        if not 'On' in obj.game.properties:
            bpy.ops.object.game_property_new()
            prop = context.object.game.properties[-1]
            prop.name = 'On'
            prop.type = 'BOOL'

        self.assignProp(context, 'On', self.On)
        
        if not 'Energy' in obj.game.properties:
            bpy.ops.object.game_property_new()
            prop = context.object.game.properties[-1]
            prop.name = 'Energy'
            prop.type = 'FLOAT'

        self.assignProp(context, 'Energy', self.energy)

        # add logic setup
        if not 'PropertyChange' in context.object.game.sensors:
            bpy.ops.logic.sensor_add(type = 'PROPERTY')
            sens = context.object.game.sensors[-1]
            sens.name = 'PropertyChange'
            sens.property = 'On'
            sens.evaluation_type = 'PROPCHANGED'

            bpy.ops.logic.controller_add(type = 'PYTHON')
            contr = context.object.game.controllers[-1]
            contr.mode = 'MODULE'
            contr.module = "lights.change_light_energy"

            contr.link(sensor = sens)

        return {'FINISHED'}
    
    def invoke(self, context, event):
        obj = context.object

        if obj.type != 'LAMP':
            print("Sorry, no lamp selected")
            self.report({'INFO'}, "Sorry, no lamp selected")
            return {'CANCELLED'}
        
        if 'On' in obj.game.properties:
            self.On = obj.game.properties['On'].value
        self.energy = obj.data.energy

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
        layout.operator("object.morse_switch_dialog")

        col = layout.column()
        col.label(text = "Presets for electric devices")
        layout.operator("object.morse_light_dialog", icon = 'LAMP_SPOT')
 

def register():
    bpy.utils.register_class(MorseObjectDialog)
    bpy.utils.register_class(MorseDoorDialog)
    bpy.utils.register_class(MorseDrawerDialog)
    bpy.utils.register_class(MorseSwitchDialog)
    bpy.utils.register_class(MorseLightDialog)
    bpy.utils.register_class(MorsePanel)
    try:
        bpy.app.handlers.scene_update_post.append(update)
    except AttributeError:
        pass
def unregister():
    bpy.utils.unregister_class(MorseObjectDialog)
    bpy.utils.unregister_class(MorseDoorDialog)
    bpy.utils.unregister_class(MorseDrawerDialog)
    bpy.utils.unregister_class(MorseSwitchDialog)
    bpy.utils.unregister_class(MorseLightDialog)
    bpy.utils.unregister_class(MorsePanel)
    try:
        bpy.app.handlers.scene_update_post.remove(update)
    except AttributeError:
        pass
    
