# -*- coding: utf-8 -*-

bl_addon_info = {
    'name': 'Panels for easy view and simple snap',
    'author': 'Izaak Van Crombrugge (KULeuven)',
    'version': '2010/09/01',
    'blender': (2, 5, 3),
    'location': 'View3D > Properties',
    'description': 'Adds 3 panels: Simple Snap, View Menu and View Numpad',
    'warning': '', # used for warning icon and text in addons panel
    'wiki_url': 'http://wiki.blender.org/index.php/Robotics:Scripts/viewPanel',
    'tracker_url': '',
    'category': 'Robotics'}

#viewPanel.py
import bpy, bgl, blf, mathutils.geometry
import mathutils
from mathutils import *
from math import *
from bpy.props import FloatVectorProperty
from bgl import *
import random

def draw_callback_px(self, context):
	'''The POST VIEW callback that draws extra geometry in the OpenGL-buffer.'''
	endW, normal = place(context)
		
	if not normal:
		#no collision found, so no display of hit point
		return
	
	endW = Vector(endW)
	normal = Vector(normal)
	
	offset = Vector((0,0,0.05))
	arrowPoint = endW + normal.normalize()
	
	actObj = context.active_object
	startW = actObj.matrix_world.translation_part()
	# endW = startW + Vector((0,0,-100))
	
	
	# Store Line width
	lineWidth_prev = bgl.Buffer(bgl.GL_FLOAT, [1])
	bgl.glGetFloatv(bgl.GL_LINE_WIDTH, lineWidth_prev)
	lineWidth_prev = lineWidth_prev[0]
	
	#set new line width
	bgl.glLineWidth(5)
	bgl.glEnable(bgl.GL_BLEND)
	bgl.glColor4f(0.0,0.8,0.0,0.5)	#line color
	
	#Draw the line
	bgl.glBegin(bgl.GL_LINE_STRIP)
	bgl.glVertex3f(startW.x, startW.y, startW.z)
	bgl.glVertex3f(endW.x, endW.y, endW.z)
	bgl.glEnd()
	
	#Draw the arrow
	bgl.glLineWidth(3)
	bgl.glColor4f(0.0,0.0,1.0,1.0)	#line color
	bgl.glBegin(bgl.GL_LINE_STRIP)
	bgl.glVertex3f(endW.x, endW.y, endW.z)
	bgl.glVertex3f(arrowPoint.x, arrowPoint.y, arrowPoint.z)
	bgl.glEnd()
	
	bgl.glColor4f(0.8,0.0,0.0,0.5)	#line color
	glCircle(endW+offset, 1.0, normal)
	
	#restore line width
	bgl.glLineWidth(lineWidth_prev)
	
def drawText(self, context):
	'''The POST PIXEL callback that draws text in the OpenGL-buffer.'''
	# draw text in the 3d-viewport
	texts = context.scene['robotics_viewPortText'].splitlines()
	texts.reverse()
	
	
	blf.size(0, 20, 72)
	for i,text in enumerate(texts):
		blf.position(0, 60, 25+i*20, 0)
		blf.draw(0, text)

def glCircle(center, radius, normal=Vector((0,0,1)), resolution = 20):
	'''A helper function to draw a glCircle.'''
	bgl.glBegin(bgl.GL_LINE_STRIP)
	#create new orhogonal axis from the normal vector
	zVec = normal
	xVec = Vector((1,0,0))
	yVec = zVec.cross(xVec).normalize()
	xVec = yVec.cross(zVec).normalize()
	mat = Matrix(xVec, yVec, zVec).transpose()
	for i in range(resolution+1):
		vec = Vector((cos(i/resolution*2*pi), sin(i/resolution*2*pi), 0))
		v = vec*mat + center
		bgl.glVertex3f(v.x, v.y, v.z)
	bgl.glEnd()

class VIEW3D_PT_robotics_simpleSnap(bpy.types.Panel):
	'''Simple Snap panel with some options to make scene building easier.'''
	bl_space_type = 'VIEW_3D' # Window type where the panel will be shown
	bl_region_type = 'UI'
	# bl_context = 'scene' # Where to show panel in space_type
	bl_label = 'Simple Snap'
	move = False
	
	def draw_header(self, context):
		layout = self.layout
		lbl = layout.label('Robotics', icon='PLUGIN')
		lbl = layout.label(icon='SNAP_ON')
	
	def draw(self, context):
		layout = self.layout
		
		#Snap
		lbl = layout.label('Snap:')
		box = layout.box()
		row = box.row(align=True)
		prop = row.prop(context.tool_settings, 'use_snap', text='Auto Snap', toggle=False)
		prop = row.prop(context.tool_settings, 'use_snap_align_rotation', toggle=False)
		row = box.row()
		prop = row.prop(context.tool_settings, 'snap_element', expand=True, icon_only=True)
		row = box.row()
		prop = row.prop(context.tool_settings, 'snap_target', expand=True, icon_only=True)
		
		# sep = layout.separator()
		
		#Place Object
		'''Start drawing'''
		lbl = layout.label('Place Object:')
		box = layout.box()
		if context.active_object:
			row = box.row()
			prop = row.prop(context.active_object, 'lock_location')
		else:
			lbl = box.label('(No active object)', icon='ERROR')
		#TODO:
		#2 options: drop with center or drop with lowest point
		row = box.row(align=True)
		op = row.operator('robotics.place_object', icon='SNAP_SURFACE')
		op = row.operator('transform.translate')

bpy.types.Scene.robotics_setViewprefs = bpy.props.BoolProperty(name='Set View Preferences')
bpy.context.scene['robotics_setViewprefs'] = False #Assign a value to make sure the property is created.
class VIEW3D_PT_robotics_viewPanel(bpy.types.Panel):
	'''View panel with easy access to handy operators for simple scene building.'''
	bl_space_type = 'VIEW_3D' # Window type where the panel will be shown
	bl_region_type = 'UI'
	# bl_context = 'scene' # Where to show panel in space_type
	bl_label = 'View Menu'
	
	def draw_header(self, context):
		layout = self.layout
		lbl = layout.label('Robotics', icon='PLUGIN')
		lbl = layout.label(icon='RESTRICT_VIEW_OFF')
	
	def draw(self, context):
		layout = self.layout
		
		'''Start drawing'''
		#Top buttons
		prop = layout.prop(context.scene, 'robotics_setViewprefs', toggle=True, icon='PREFERENCES')
		if context.scene['robotics_setViewprefs']:
			box = layout.box()
			prop = box.prop(bpy.context.user_preferences.view, 'use_zoom_to_mouse')
			prop = box.prop(bpy.context.user_preferences.view, 'use_rotate_around_active')
			prop = box.prop(bpy.context.user_preferences.view, 'use_auto_perspective')
			row = box.row()
			lbl = row.label('Orbit Style:')
			prop = row.prop(bpy.context.user_preferences.inputs, 'view_rotate_method', expand=True)
			op = box.operator('robotics.set_viewprefs')
			
		row = layout.row(align=True)
		op = row.operator('robotics.view_all')
		op = row.operator('view3d.zoom_border', icon='ZOOM_SELECTED')
		row = layout.row()
		op = row.operator('view3d.fly', icon='BOIDS')
		op = row.operator('view3d.fly_help', icon='HELP')

class VIEW3D_PT_robotics_viewNumpad(bpy.types.Panel):
	'''View Numpad panel that shows the same layout as the numpad, can make learning the numpad view commands easier.'''
	bl_space_type = 'VIEW_3D' # Window type where the panel will be shown
	bl_region_type = 'UI'
	# bl_context = 'scene' # Where to show panel in space_type
	bl_label = 'View Numpad'
	
	scene = bpy.context.scene
	bpy.types.Scene.robotics_showText = bpy.props.BoolProperty(name='Show text')
	scene['robotics_showText'] = True
	bpy.types.Scene.robotics_buttonsConnected = bpy.props.BoolProperty(name='Connect buttons')
	scene['robotics_buttonsConnected'] = True
	
	def draw_header(self, context):
		layout = self.layout
		lbl = layout.label('Robotics', icon='PLUGIN')
		lbl = layout.label(icon='MESH_GRID')
		
	def draw(self, context):
		layout = self.layout
		
		'''Check properties for checkbox'''
		if context.area.active_space.local_view:
			localIcon = 'CHECKBOX_HLT'
		else:
			localIcon = 'CHECKBOX_DEHLT'
		
		'''Conditional markup: hide/show text'''
		if not context.scene.robotics_showText:
			left,right,up,down = ' ',' ',' ',' '
		else:
			left,right,up,down = 'left','right','up','down'
		
		'''Start drawing'''
		
		#First block
		connected = context.scene['robotics_buttonsConnected']
		
		col = layout.column(align=connected)
		row = col.row(align=connected)
		op = row.operator('robotics.view_top')
		op = row.operator('robotics.orbit_up', icon='TRIA_UP', text=up)
		op = row.operator('view3d.localview', icon=localIcon)
		row = col.row(align=connected)
		op = row.operator('robotics.orbit_left', icon='TRIA_LEFT', text=left)
		op = row.operator('view3d.view_persportho')
		op = row.operator('robotics.orbit_right', icon='TRIA_RIGHT', text=right)
		row = col.row(align=connected)
		op = row.operator('robotics.view_front')
		op = row.operator('robotics.orbit_down', icon='TRIA_DOWN', text=down)
		op = row.operator('robotics.view_right')
		row = col.row(align=connected)
		op = row.operator('robotics.view_camera')
		op = row.operator('view3d.view_selected')
		
		sep = layout.separator()
		#second block
		col = layout.column(align=connected)
		row = col.row(align=connected)
		op = row.operator('robotics.view_bottom')
		op = row.operator('robotics.pan_up', icon='TRIA_UP', text=up)
		op = row.operator('robotics.dummy_ctrl', icon='INFO')
		row = col.row(align=connected)
		op = row.operator('robotics.pan_left', icon='TRIA_LEFT', text=left)
		op = row.operator('robotics.dummy_none')
		# op = row.operator('robotics.set_viewprefs')
		op = row.operator('robotics.pan_right', icon='TRIA_RIGHT', text=right)
		row = col.row(align=connected)
		op = row.operator('robotics.view_back')
		op = row.operator('robotics.pan_down', icon='TRIA_DOWN', text=down)
		op = row.operator('robotics.view_left')
		row = col.row(align=connected)
		op = row.operator('view3d.object_as_camera')
		op = row.operator('view3d.view_center_cursor')

		sep = layout.separator()
		
		#draw option buttons
		row = layout.row(align=True)
		prop = row.prop(context.scene, 'robotics_showText', toggle=True)
		prop = row.prop(context.scene, 'robotics_buttonsConnected', toggle=True)

class ROBOTICS_OT_place_object(bpy.types.Operator):
	'''Find the closest point under the active object that intersects a mesh that has the property "robotics_ground" set to True and place the active object at this point.'''
	bl_idname = "ROBOTICS_OT_place_object"
	bl_label = "Place Object"
	__doc__ = "place object on ground"
	
	offset = FloatVectorProperty(name="Offset", size=3)
	move = False
	
	@classmethod
	def poll(cls, context):
		return (context.active_object != None) and (not bpy.types.ROBOTICS_OT_place_object.move)
	
	def modal(self, context, event):
		
		if context.area:
			context.area.tag_redraw()
		
		if event.type in ('RIGHTMOUSE', 'ESC'):
			context.active_object.location = self._initial_location
			self.exit(context)
			print('CANCELLED')
			return {'CANCELLED'}
			
		
		# if event.type == 'G':
			# context.scene['robotics_viewPortText'] = 'ENTER to confirm\nESC to cancel'
			
		if event.type in ('RET', 'NUMPAD_ENTER', 'SPACE'):
			actObj = context.active_object
			min, normal = place(context)
			placeObject(min, actObj)
			
			self.exit(context)
			
			bpy.ops.transform.translate()
			print('FINISHED')
			return {'FINISHED'}
		

		return {'PASS_THROUGH'}
		
	def invoke(self, context, event):
		if context.area.type == 'VIEW_3D':
			# start script
			context.windowmanager.add_modal_handler(self)
			context.scene['robotics_viewPortText'] = ''\
				'To move object:\n'\
				'    G (button Translate)\n'\
				'To confirm:\n'\
				'    ENTER or SPACE\n'\
				'To cancel:\n'\
				'    ESC or RIGHTMOUSE'
			self._handle1 = context.region.callback_add(draw_callback_px, (self, context), 'POST_VIEW')
			self._handle2 = context.region.callback_add(drawText, (self, context), 'POST_PIXEL')
			
			self._initial_location = context.active_object.location.copy()
			bpy.types.ROBOTICS_OT_place_object.move = True
			print('RUNNING_MODAL')
			return {'RUNNING_MODAL'}
			
		else:
			# operator not invoked from within the 3d-view
			self.report({'WARNING'}, "View3D not found, cannot run operator")
			return {'CANCELLED'}
	
	def execute(self, context):
		actObj = context.active_object
		min, normal = place(context)
		placeObject(min, actObj)
		print('execute')
		return {'FINISHED'}
	
	def exit(self, context):
		# stop script
		context.region.callback_remove(self._handle1)
		context.region.callback_remove(self._handle2)
		bpy.types.ROBOTICS_OT_place_object.move = False


def place(context):
	'''Find the closest point under the active object that intersects a mesh.
	@return: (hitpoint, hit normal)
	@returntype: (Vector, Vector)'''
	ground = None
	normW = None
	pointList = []
	normalList = []
	actObj = context.active_object
	
	groundList = []
	layers = []
	for i in range(20):
		if context.scene.layers[i]:
			layers.append(i)
			
	for obj in context.scene.objects:
		#only mesh objects on the visual layers are good ground objects.
		if obj.type != 'MESH' or obj == context.active_object:
			continue
		for i in layers:
			if obj.layers[i]:
				groundList.append(obj)
				break
	
	
	for ground in groundList:
		# if 'robotics_ground' in ground and ground['robotics_ground']:
		if True:
			#start is center of active object
			startPtW = localToWorld(Vector((0,0,0)), actObj)
			endPtW = startPtW - Vector((0,0,100))
			
			#Translate the start point to local coordinates for the ground object
			startPtL = worldToLocal(startPtW, ground)
			endPtL = worldToLocal(endPtW, ground)
			
			#Find intersection
			locL, normL, index = ground.ray_cast(startPtL, endPtL)
			if index != -1:
				locW = localToWorld(locL, ground)
				normW = normalLocalToWorld(normL, ground)
				
				pointList.append(locW)
				normalList.append(normW)
				
	obPos = actObj.matrix_world.translation_part()
	if len(pointList):
		min = pointList[0]
		norm = normalList[0]
	else:
		min = obPos
		norm = None
	for i, point in enumerate(pointList):
		if dist(point, obPos) < dist(min, obPos):
			min = point
			norm = normalList[i]
	
	return (min, norm)

def invertScale(matrix):
	'''Invert the scale of a given transform matrix.
	@return: A NEW transform matrix with the inverted scale of the givenone.'''
	m = Matrix(matrix)
	m[0] /= m[0].magnitude**2
	m[1] /= m[1].magnitude**2
	m[2] /= m[2].magnitude**2
	return m
	
def worldToLocal(vect, obj):
	'''Convert the given world vector to a local vector in the objects local coordinates (usually used for a location vector).
	@arg vect: local vector
	@type vect: Vector
	@arg obj: reference object
	@type obj: Blender Object
	@return: A new local vector
	@returntype: Vector'''
	m = invertScale(obj.matrix_world)
	return (vect - m.translation_part()) * m
def normalWorldToLocal(vect, obj):
	'''Convert the given world vector to a local vector in the objects local coordinates WITHOUT adding the translation of the reference object (usually used for a direction vector, like a normal).
	@arg vect: local vector
	@type vect: Vector
	@arg obj: reference object
	@type obj: Blender Object
	@return: A new local vector
	@returntype: Vector'''
	m = invertScale(obj.matrix_world)
	return vect * m

def localToWorld(vect, obj):
	'''Same as L{worldToLocal} but inverse.
	@return: A new world vector
	@returntype: Vector'''
	m = invertScale(obj.matrix_world)
	n = Matrix(m).invert()
	return vect * n + m.translation_part()
def normalLocalToWorld(vect, obj):
	'''Same as L{normalWorldToLocal} but inverse.
	@return: A new world vector
	@returntype: Vector'''
	m = invertScale(obj.matrix_world)
	n = Matrix(m).invert()
	return vect * n
def placeObject(vect, obj):
	'''Place a given object on a given world location.
	@arg vect: world location
	@arg obj: object to place'''
	#NOTE: resize4D() works "in place", so the object is modified,
	#therefore a copy is made, or a returned result (=new object) is used.
	if obj.parent:
		obj.matrix_local[3] = worldToLocal(vect, obj.parent).resize4D()
	else:
		obj.matrix_world[3] = Vector(vect).resize4D()

def dist(v1, v2):
	'''Distance between endpoints of 2 vectors.
	@return: (v1-v2).magnitude
	@returntype: Integer'''
	return (v1-v2).magnitude

def mul(a,b):
	'''Takes 2 Vectors and multiplies the corresponding items.
	@return: Vector((a.x*b.x, a.y*b.y, a.z*b.z))
	@returntype: Vector'''
	return Vector((a.x*b.x, a.y*b.y, a.z*b.z))
def div(a,b):
	'''Takes 2 Vectors and divides the corresponding items.
	@return: Vector((a.x/b.x, a.y/b.y, a.z/b.z))
	@returntype: Vector'''
	return Vector((a.x/b.x, a.y/b.y, a.z/b.z))
def div2(a,b):
	'''Takes 2 Vectors and divides the corresponding items twice.
	@return: Vector((a.x/b.x**2, a.y/b.y**2, a.z/b.z**2))
	@returntype: Vector'''
	return Vector((a.x/b.x**2, a.y/b.y**2, a.z/b.z**2))
def inv(a):
	'''Invert every element of a given Vector.
	@return: Vector((1/a.x, 1/a.y, 1/a.z)) (a new instance)
	@returntype: Vector'''
	return Vector((1/a.x, 1/a.y, 1/a.z))
		
class VIEW3D_OT_fly_help(bpy.types.Operator):
	'''Link to the wiki with help on the use of Fly Navigation.'''
	bl_idname = "VIEW3D_OT_fly_help"
	bl_label = ""
	__doc__ = "Link to wiki: how to use Fly Navigation"
	
	def execute(self, context):
		bpy.ops.wm.url_open(url='http://wiki.blender.org/index.php/Doc:Manual/3D_interaction/Navigating#Fly_mode')
		return {'FINISHED'}

class ROBOTICS_OT_view_top(bpy.types.Operator):
	'''All ROBOTICS_OT_view_, ROBOTICS_OT_orbit_ and ROBOTICS_OT_pan_ classes are simple wrappers for the commands of the numpad'''
	bl_idname = "ROBOTICS_OT_view_top"
	bl_label = "Top"
	__doc__ = "Top view (num 7)"
	
	def execute(self, context):
		bpy.ops.view3d.viewnumpad(type='TOP')
		return {'FINISHED'}
class ROBOTICS_OT_view_front(bpy.types.Operator):
	bl_idname = "ROBOTICS_OT_view_front"
	bl_label = "Front"
	__doc__ = "Front view (num 1)"
	
	def execute(self, context):
		bpy.ops.view3d.viewnumpad(type='FRONT')
		return {'FINISHED'}
class ROBOTICS_OT_view_right(bpy.types.Operator):
	bl_idname = "ROBOTICS_OT_view_right"
	bl_label = "Right"
	__doc__ = "Right view (num 3)"
	
	def execute(self, context):
		bpy.ops.view3d.viewnumpad(type='RIGHT')
		return {'FINISHED'}
class ROBOTICS_OT_view_bottom(bpy.types.Operator):
	bl_idname = "ROBOTICS_OT_view_bottom"
	bl_label = "Bottom"
	__doc__ = "Bottom view (ctrl + num 7)"
	
	def execute(self, context):
		bpy.ops.view3d.viewnumpad(type='BOTTOM')
		return {'FINISHED'}
class ROBOTICS_OT_view_back(bpy.types.Operator):
	bl_idname = "ROBOTICS_OT_view_back"
	bl_label = "Back"
	__doc__ = "Back view (ctrl + num 1)"
	
	def execute(self, context):
		bpy.ops.view3d.viewnumpad(type='BACK')
		return {'FINISHED'}
class ROBOTICS_OT_view_left(bpy.types.Operator):
	bl_idname = "ROBOTICS_OT_view_left"
	bl_label = "Left"
	__doc__ = "Left view (ctrl + num 3)"
	
	def execute(self, context):
		bpy.ops.view3d.viewnumpad(type='LEFT')
		return {'FINISHED'}

class ROBOTICS_OT_view_camera(bpy.types.Operator):
	bl_idname = "ROBOTICS_OT_view_camera"
	bl_label = "Camera"
	__doc__ = "Camera view (num 0)"
	
	def execute(self, context):
		bpy.ops.view3d.viewnumpad(type='CAMERA')
		return {'FINISHED'}

class ROBOTICS_OT_orbit_left(bpy.types.Operator):
	bl_idname = "ROBOTICS_OT_orbit_left"
	bl_label = ""
	__doc__ = "Orbit left (num 4)"
	
	def execute(self, context):
		bpy.ops.view3d.view_orbit(type='ORBITLEFT')
		return {'FINISHED'}
class ROBOTICS_OT_orbit_right(bpy.types.Operator):
	bl_idname = "ROBOTICS_OT_orbit_right"
	bl_label = ""
	__doc__ = "Orbit right (num 6)"
	
	def execute(self, context):
		bpy.ops.view3d.view_orbit(type='ORBITRIGHT')
		return {'FINISHED'}
class ROBOTICS_OT_orbit_up(bpy.types.Operator):
	bl_idname = "ROBOTICS_OT_orbit_up"
	bl_label = ""
	__doc__ = "Orbit up (num 8)"
	
	def execute(self, context):
		bpy.ops.view3d.view_orbit(type='ORBITUP')
		return {'FINISHED'}
class ROBOTICS_OT_orbit_down(bpy.types.Operator):
	bl_idname = "ROBOTICS_OT_orbit_down"
	bl_label = ""
	__doc__ = "Orbit down (num 2)"
	
	def execute(self, context):
		bpy.ops.view3d.view_orbit(type='ORBITDOWN')
		return {'FINISHED'}

class ROBOTICS_OT_pan_left(bpy.types.Operator):
	bl_idname = "ROBOTICS_OT_pan_left"
	bl_label = ""
	__doc__ = "Pan left (ctrl + num 4)"
	
	def execute(self, context):
		bpy.ops.view3d.view_pan(type='PANLEFT')
		return {'FINISHED'}
class ROBOTICS_OT_pan_right(bpy.types.Operator):
	bl_idname = "ROBOTICS_OT_pan_right"
	bl_label = ""
	__doc__ = "Pan right (ctrl + num 6)"
	
	def execute(self, context):
		bpy.ops.view3d.view_pan(type='PANRIGHT')
		return {'FINISHED'}
class ROBOTICS_OT_pan_up(bpy.types.Operator):
	bl_idname = "ROBOTICS_OT_pan_up"
	bl_label = ""
	__doc__ = "Pan up (ctrl + num 8)"
	
	def execute(self, context):
		bpy.ops.view3d.view_pan(type='PANUP')
		return {'FINISHED'}
class ROBOTICS_OT_pan_down(bpy.types.Operator):
	bl_idname = "ROBOTICS_OT_pan_down"
	bl_label = ""
	__doc__ = "Pan down (ctrl + num 2)"
	
	def execute(self, context):
		bpy.ops.view3d.view_pan(type='PANDOWN')
		return {'FINISHED'}

class ROBOTICS_OT_dummy_ctrl(bpy.types.Operator):
	'''Just a dummy Operator to make it possible to place a button'''
	bl_idname = "ROBOTICS_OT_dummy_ctrl"
	bl_label = "(ctrl)"
	__doc__ = "This block shows the controls when holding the ctrl-key"
	
	@classmethod
	def poll(cls, context):
		return False
	
class ROBOTICS_OT_dummy_none(bpy.types.Operator):
	'''Just a dummy Operator to make it possible to place a button'''
	bl_idname = "ROBOTICS_OT_dummy_none"
	bl_label = " "
	__doc__ = ""
	
	@classmethod
	def poll(cls, context):
		return False
	
	
class ROBOTICS_OT_view_all(bpy.types.Operator):
	'''Self defined view_all Operator because bpy.ops.view3d.view_all() is not callable from the panel. (Context requirements are too strict)'''
	bl_idname = "ROBOTICS_OT_view_all"
	bl_label = "View all"
	__doc__ = "Show all objects in scene (Home)"
	
	def execute(self, context):
		actObj, selection = store(context)
		#act
		bpy.ops.object.select_all(action='SELECT')
		bpy.ops.view3d.view_selected()
		restore(context, actObj, selection)
		
		return {'FINISHED'}

def store(context):
	'''"Store" the selected object and active object by passing it as a return. This is usefull to store the selection before an operator and restoring it at the end.
	@return: active object, selection
	@returntype: ( blender object, list of blender objects )'''
	actObj = context.active_object
	selection = context.selected_objects
	return actObj, selection

def restore(context, actObj, selection):
	'''"Restore" the selected object and active object by setting the selection.
	@see: L{store}
	@arg actObj: object to be the active object
	@arg selection: list of blender objects that have to be selected'''
	bpy.ops.object.select_all(action='DESELECT')
	for obj in context.scene.objects:
		if obj in selection:
			obj.select = True
	context.scene.objects.active = actObj
	
	
class ROBOTICS_OT_set_viewprefs(bpy.types.Operator):
	'''Set some user preferences to easy to use controls.'''
	bl_idname = "ROBOTICS_OT_set_viewprefs"
	bl_label = "Set Optimal Viewpreferences"
	__doc__ = "Set some user preferences to easy to use controls."
	
	def execute(self, context):
		v = bpy.context.user_preferences.view
		v.use_zoom_to_mouse = True
		v.use_rotate_around_active = True
		v.use_auto_perspective = True
		bpy.context.user_preferences.inputs.view_rotate_method = 'TURNTABLE'
		return {'FINISHED'}
	
	
classes = [
	ROBOTICS_OT_place_object,

	VIEW3D_OT_fly_help,

	ROBOTICS_OT_view_top,
	ROBOTICS_OT_view_front,
	ROBOTICS_OT_view_right,
	ROBOTICS_OT_view_bottom,
	ROBOTICS_OT_view_back,
	ROBOTICS_OT_view_left,
	ROBOTICS_OT_view_camera,

	ROBOTICS_OT_orbit_left,
	ROBOTICS_OT_orbit_right,
	ROBOTICS_OT_orbit_up,
	ROBOTICS_OT_orbit_down,

	ROBOTICS_OT_pan_left,
	ROBOTICS_OT_pan_right,
	ROBOTICS_OT_pan_up,
	ROBOTICS_OT_pan_down,

	ROBOTICS_OT_dummy_ctrl,
	ROBOTICS_OT_dummy_none,

	ROBOTICS_OT_set_viewprefs,
	ROBOTICS_OT_view_all,
	
	VIEW3D_PT_robotics_simpleSnap,
	VIEW3D_PT_robotics_viewPanel,
	VIEW3D_PT_robotics_viewNumpad
]
def register():
	pass
	'''Registers the needed classes. (Needed to turn addon on and off)'''
	# for cl in classes:
		# bpy.types.register(cl)

def unregister():
	pass
	'''Unregisters the needed classes. (Needed to turn addon on and off)'''
	# for cl in classes:
		# bpy.types.unregister(cl)

if __name__ == "__main__":
    register()