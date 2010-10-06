import Rasterizer
import GameLogic
import GameKeys
import Blender
import math

import Mathutils as mathutils


#import GameTypes

# get the controller the script it attached to
controller = GameLogic.getCurrentController()
# get a list of sensors connected to the python controller
senList = controller.sensors
# get a list of actuators connected to the python controller
actList = controller.actuators

MAX_HEAD_PAN = 60.0 #in deg
MAX_HEAD_TILT = 60.0
MAX_HEAD_YAW = 30.0	
MAX_BODY = 180


#arm = Blender.Armature.Get("Armature")
#bone = arm.getBones()[0]
#print dir(bone)
	
''' RuntimeError: Blender.Armature - Error: Armature_CreatePyObject: Internal Error Ocurred
arm = Blender.Armature.Get("Armature")
print dir(arm) 
'''



def rotateHead(contr):
	
	# get the object this script is attached to
	head = contr.owner
	

	
	status = head['statusHead']
	
	# get the current scene
	scene = GameLogic.getCurrentScene()
	# get list of objects in scene
	objList = scene.objects
	# get OBCube from objList.  No default value
	body = objList.get("OBArmature")
	# get the orientation of the body
	chest = scene.objects['OBChest']
	
	

	# Get sensor named Mouse
	keyboard = contr.sensors['TurnHead']
	keylist = keyboard.events
	for key in keylist:
		# key[0] == GameKeys.keycode, key[1] = status
		if key[1] == GameLogic.KX_INPUT_ACTIVE:
			if key[0] == GameKeys.HKEY:
				status = True
				
			
	""" Read the movements of the mouse and apply them
		as a rotation to the head. """

	# Get sensor named Mouse
	mouse = contr.sensors['Mouse']

	# get width and height of game window
	width = Rasterizer.getWindowWidth()
	height = Rasterizer.getWindowHeight()
	
	# get mouse movement from function
	move = mouse_move(head, mouse, width, height)
	
	# set mouse sensitivity
#	sensitivity = head['Sensitivity']
	
	# Amount, direction and sensitivity
	leftRight = move[0] *0.01 #* sensitivity
	upDown = move[1] *0.01 #* sensitivity

	#TODO: Could be improved: we don't control the YAW -> 		the head may have a strange position!
	
	h = head.orientation
	c = chest.orientation
	
	hRoll = math.atan2(h[1][0], h[0][0]) * 180 / math.pi
	hPitch = -math.asin(h[2][0]) * 180 / math.pi
	#yaw = math.atan2(m[2][1], m[2][2]) * 180 / math.pi
	#print [roll, pitch, yaw]
	
	cRoll = math.atan2(c[1][0], c[0][0]) * 180 / math.pi
	cPitch = -math.asin(c[2][0]) * 180 / math.pi

	if status == True:
		# set the values

		rRoll = hRoll - cRoll
		
		#elif (roll + leftRight) < (roll):
		#	head.applyRotation( [0.0, 0.0, leftRight], True )
			
		print("%.4f <= %.4f <= %.4f | %.4f" % ((cRoll - MAX_HEAD_PAN), (hRoll), (cRoll + MAX_HEAD_PAN), leftRight))
			
		if  (-MAX_HEAD_PAN) <= (rRoll) <= (MAX_HEAD_PAN):		
			head.applyRotation( [0.0, 0.0, leftRight], True)
		elif (rRoll < -MAX_HEAD_PAN and leftRight > 0):
			head.applyRotation( [0.0, 0.0, leftRight], True)
		elif (rRoll > MAX_HEAD_PAN and leftRight < 0):
			head.applyRotation( [0.0, 0.0, leftRight], True)
		else:
			body.applyRotation( [0.0, 0.0, leftRight], True)
			#if  (cRoll - MAX_HEAD_PAN-1) <= (hRoll + leftRight) <= (cRoll + MAX_HEAD_PAN+1):
				#head.applyRotation( [0.0, 0.0, -leftRight], True)
		
			
	# Center mouse in game window
	Rasterizer.setMousePosition(width/2, height/2)
	
	
	# define mouse movement function
def mouse_move(human, mouse, width, height):

	""" Get the movement of the mouse as an X, Y coordinate. """
	# distance moved from screen center
	x = width/2 - mouse.position[0]
	y = height/2 - mouse.position[1]


	# intialize mouse so it doesn't jerk first time
	try:
		human['mouseInit']
	except KeyError:
		x = 0
		y = 0
		# bug in Add Property
		# can't use True.  Have to use 1
		human['mouseInit'] = 1
		
	# return mouse movement
	return (x, y)