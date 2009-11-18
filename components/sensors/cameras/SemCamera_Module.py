'''
This module implement a "semantic camera" sensor for the OpenRobots Simulator.

This special camera returns the list of objects as seen by the robot's cameras,
with unique id, possibly (if set in the objects' properties) the type of object
and the colour of the object.

Other such high-level information (the semantic description of the scene) can be
added.

Version: 1.0
Date: 16 Nov. 2009
Author: Séverin Lemaignan <severin.lemaignan@laas.fr>

Copyright LAAS-CNRS 2009
'''

import sys, os
import GameLogic
import array, struct

#import the ontology server proxy
import oro

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

# Definition of global variables
ob = ''
port_name = ''
structObject = ''


def init(contr):
	global ob
	global port_name
	global structObject

	print '######## SEMANTIC CAMERA INITIALIZATION ########'
	print

	# To get the game object this controller is on:
	ob = contr.owner
	parent = ob.parent
	if not parent:
		parent = ob

	port_name = '{0}/{1}/clean'.format(parent.name, ob['Component_Type'])

	# Middleware initialization
	if not hasattr(GameLogic, 'orsConnector'):
		GameLogic.orsConnector = MiddlewareConnector()
		
	#Create Connection port
	try:
		#GameLogic.orsConnector.registerBufferedPortImageRgb([port_name])
		#GameLogic.orsConnector.registerBufferedPortBottle([port_name])
		GameLogic.orsConnector.registerPort([port_name])
	except NotImplementedError as detail:
		print "ERROR: Unable to create the port:"
		print detail

	# Create a key for a dictionary of cameras
	#  necesary if there are more than one camera added to the scene
	key = 'SemCamera'
	screen_name = 'OBSemCameraCube'
	camera_name = 'OBCameraRobot'

	name_len = len(ob.name)
	if name_len > 4 and ob.name.endswith('.00', name_len-4, name_len-1):
		extension = ob.name[name_len-4:]
		key = key + extension
		screen_name = screen_name + extension
		camera_name = camera_name + extension
	# Store the key as an ID in the Empty object
	ob['camID'] = key
	print "KEY BEING USED IS: '{0}'".format(key)

	# Get the reference to the camera and screen
	scene = GameLogic.getCurrentScene()
	screen = scene.objects[screen_name]
	camera = scene.objects[camera_name]
	
	#trackedObject is a dictionary containing the list of tracked objects 
	#(->meshes with a class property set up) as keys and the bounding boxes of 
	#these objects as value.
	#TODO: Use the robot dictionnary
	if not hasattr(GameLogic, 'trackedObjects'):
		print ' * Initialization of trackedObjects variable...'
		GameLogic.trackedObjects = dict.fromkeys([ obj for obj in scene.objects if obj.getPropertyNames().count('objClass')!=0 ])
		
		for obj in GameLogic.trackedObjects.keys():
			GameLogic.trackedObjects[obj] = Blender.Object.Get(obj.name[2:]).getBoundBox(0) #getBoundBox(0) returns the bounding box in local space instead of world space.
			print '  - ',obj.name

	#visibleObjects stores the list of currently visible objects by the robot.
	#TODO: Use the robot dictionnary
	if not hasattr(GameLogic, 'visibleObjects'):
		print ' * Initialization of visibleObjects variable...'
		GameLogic.visibleObjects = []


	ob['Init_OK'] = True

#print_properties(ob)
def print_properties(ob):
	# Read the list of properties
	properties = ob.getPropertyNames()

	print "Properties of object: ", ob
	for prop in properties:
		print prop, " = ", ob[prop]


def grab():
	'''Do the actual semantic 'grab': iterate over all the tracked objects, and 
	check if they are visible for the robot. Send messages accordingly.
	'''

	if ob['Init_OK']:
		# execute only when the 'grab_image' key is released
		# (if we don't test that, the code get executed two times,
		#	when pressed, and when released)
		sensor = GameLogic.getCurrentController().sensors['Check_capturing']

		if sensor.positive:
			for obj in trackedObjects:
				#if the object is visible and no yet in the visibleObjects list...
				if checkVisible(obj) and visibleObjects.count(obj) == 0:
					visibleObjects.append(obj)
					print obj.name, '(', obj['objClass'],', hue:',retrieveHue(obj),') just appeared.'

					#...and send it
					#GameLogic.orsConnector.postMessage(,port_name)

				#if the object is not visible but was in the visibleObjects list...
				if not checkVisible(obj) and visibleObjects.count(obj) != 0:
					visibleObjects.remove(obj)
					print obj.name + '(' + obj['objClass'] + ') just disappeared.'

					#...and send it
					#GameLogic.orsConnector.postMessage(,port_name)
			
			
#the main method: check is an object lies inside of the camera frustrum. 	
def checkVisible(obj):
	# camera inside box?
	
	bb = trackedObjects[obj] #trackedObjects was filled at initialization with the object's bounding boxes
	pos = obj.position
	
	#translate the bounding box to the current object position and check if it is in the frustrum
	if camera.boxInsideFrustum([[bb_corner[i] + pos[i] for i in range(3)] for bb_corner in bb]) != camera.OUTSIDE:
	# object is inside
		return True


