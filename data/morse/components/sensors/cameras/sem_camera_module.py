'''
This module implement a "semantic camera" sensor for the OpenRobots Simulator.

This special camera returns the list of objects as seen by the robot's cameras,
with unique id, possibly (if set in the objects' properties) the type of object
and the colour of the object.

Other such high-level information (the semantic description of the scene) can be
added.

Version: 1.0
Date: 16 Nov. 2009
Author: Severin Lemaignan <severin.lemaignan@laas.fr>

Copyright LAAS-CNRS 2009
'''

import sys, os
import GameLogic
import array, struct

import Blender
#import bpy.types
# Import the ontology server proxy
#import oro

try:
   scriptRoot = os.path.join(os.environ['ORS_ROOT'],'scripts')
except KeyError:
   scriptRoot = '.'

try:
   libRoot = os.path.join(os.environ['ORS_ROOT'],'lib')
except KeyError:
   libRoot = '.'

if scriptRoot not in sys.path:
	sys.path.append(scriptRoot)
if scriptRoot not in sys.path:
	sys.path.append(libRoot)

from middleware.independent.IndependentBlender import *
import helpers.Colors
import setup.ObjectData


def init(contr):
	# Middleware initialization
	if not hasattr(GameLogic, 'orsConnector'):
		GameLogic.orsConnector = MiddlewareConnector()

	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)
	port_name = port_name + "/semantic"

	ob['Init_OK'] = False

	try:
		# Get the dictionary for the component's state
		robot_state_dict = GameLogic.robotDict[parent]
		#state_dict = GameLogic.componentDict[ob]
		ob['Init_OK'] = True
	except AttributeError:
		print ("Component Dictionary not found!")
		print ("This component must be part of a scene")

	if ob['Init_OK']:

		print ('######## SEMANTIC CAMERA INITIALIZATION ########')
		#Create Connection port
		try:
			GameLogic.orsConnector.registerPort([port_name])
		except NotImplementedError as detail:
			print ("Semantics ERROR: Unable to create the port:")
			print (detail)

		# Find the blender camera linked to the Semantic camera
		for child in ob.children:
			try:
				# Check if the object is a camera
				# Only if it is, will it have the 'frustum_culling' variable
				if child.lens:
					robot_state_dict['SemCamera'] = child
					print ("Semantics: Using camera '{0}'".format(child))
					#print ("  Camera Near Clip: {0}".format(child.near))
					#print ("  Camera Far Clip:  {0}".format(child.far))
					break
			except AttributeError as detail:
				pass


		# TrackedObject is a dictionary containing the list of tracked objects 
		# (->meshes with a class property set up) as keys
		#  and the bounding boxes of these objects as value.
		if not hasattr(GameLogic, 'trackedObjects'):
			print ('  ### Initialization of trackedObjects variable...')
			scene = GameLogic.getCurrentScene()
			GameLogic.trackedObjects = dict.fromkeys([ obj for obj in scene.objects if obj.getPropertyNames().count('objClass')!=0 ])
			
			# Store the bounding box of the marked objects
			################## WARNING ################## 
			# NOTE: This uses the Blender library, which has been removed
			#  in Blender 2.5. Thus this will likely break with the new version.
			for obj in GameLogic.trackedObjects.keys():
				# GetBoundBox(0) returns the bounding box in local space
				#  instead of world space.
				#GameLogic.trackedObjects[obj] = bpy.types.Object(obj).bound_box
				GameLogic.trackedObjects[obj] = Blender.Object.Get(obj.name[2:]).getBoundBox(0)
				print ('    - {0}'.format(obj.name))

		# VisibleObjects stores the list of currently visible objects
		#  by each independent robot.
		print ('  ### Initialization of visibleObjects list...')
		robot_state_dict['visibleObjects'] = []

		print ('######## SEMANTIC CAMERA INITIALIZED ########')


def print_properties(ob):
	# Read the list of properties
	properties = ob.getPropertyNames()

	print ("Properties of object: {0}".format(ob))
	for prop in properties:
		print ("\t{0} = {1}".format(prop, ob[prop]))


def grab(contr):
	'''Do the actual semantic 'grab': iterate over all the tracked objects, and 
	check if they are visible for the robot. Send messages accordingly.
	'''

	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)
	port_name = port_name + "/semantic"

	if ob['Init_OK']:
		robot_state_dict = GameLogic.robotDict[parent]
		camera = robot_state_dict['SemCamera']
		visibles = robot_state_dict['visibleObjects']

		# execute only when the 'grab_image' key is released
		# (if we don't test that, the code get executed two times,
		#	when pressed, and when released)
		sensor = contr.sensors['Check_capturing']

		if sensor.positive:
			for obj in GameLogic.trackedObjects:
				# If the object is visible and not yet in the visibleObjects list...
				if checkVisible(obj, camera) and visibles.count(obj) == 0:
					robot_state_dict['visibleObjects'].append(obj)
					print ("Semantic: {0}, ({1}, {2}) just appeared".format(obj.name, obj['objClass'], helpers.Colors.retrieveHue(obj)))

					#...and send it
					#GameLogic.orsConnector.postMessage(,port_name)

				# If the object is not visible and was in the visibleObjects list...
				if not checkVisible(obj, camera) and visibles.count(obj) != 0:
					robot_state_dict['visibleObjects'].remove(obj)
					print ("Semantic: {0}, ({1}) just disappeared".format(obj.name, obj['objClass']))

					#...and send it
					#GameLogic.orsConnector.postMessage(,port_name)
			
			
def checkVisible(obj, camera):
	""" Check if an object lies inside of the camera frustrum. """

	# TrackedObjects was filled at initialization
	#  with the object's bounding boxes
	bb = GameLogic.trackedObjects[obj]
	pos = obj.position
	
	#print ("\n--- NEW TEST ---")
	#print ("OBJECT {0} AT {1}".format(obj, pos))
	#print ("BBOX: >{0}<".format([[bb_corner[i] + pos[i] for i in range(3)] for bb_corner in bb]))
	#print ("BBOX: {0}".format(bb))

	# Translate the bounding box to the current object position
	#  and check if it is in the frustrum
	if camera.boxInsideFrustum([[bb_corner[i] + pos[i] for i in range(3)] for bb_corner in bb]) != camera.OUTSIDE:
	# object is inside
		return True

	return False
