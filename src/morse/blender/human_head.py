import Rasterizer
import GameLogic
import GameKeys
import math
import sys
if sys.version_info<(3,0,0):
	import Mathutils as mathutils
else:
	import mathutils

# Defined in degrees, but used in radians
MAX_HEAD_YAW = math.radians(60.0)
MAX_HEAD_PITCH = math.radians(10.0)
MAX_HEAD_ROLL = math.radians(40.0)


def rotateHead(contr):
	""" Read the movements of the mouse and apply them
		as a rotation to the head. """
	# get the object this script is attached to
	head = contr.owner
	status = head['statusHead']

	# get the current scene
	scene = GameLogic.getCurrentScene()

	# Get the body and head objects
	if sys.version_info<(3,0,0):
		body = scene.objects['OBArmature']
		chest = scene.objects['OBChest']
	else:
		body = scene.objects['Achille_Armature']
		chest = scene.objects['Chest']

	# Toggle the control of the head when the user holds the H key
	keyboard = contr.sensors['TurnHead']
	keylist = keyboard.events
	for key in keylist:
		# key[0] == GameKeys.keycode, key[1] = status
		if key[1] == GameLogic.KX_INPUT_ACTIVE:
			if key[0] == GameKeys.HKEY:
				status = True

	# Get sensor named Mouse
	mouse = contr.sensors['Mouse']

	# get width and height of game window
	width = Rasterizer.getWindowWidth()
	height = Rasterizer.getWindowHeight()

	# set mouse sensitivity
#	sensitivity = head['Sensitivity']
	sensitivity = 0.01

	# get mouse movement from function
	move = mouse_move(head, mouse, width, height)
	leftRight = move[0] * sensitivity
	upDown = move[1] * sensitivity

	h = head.orientation
	c = chest.orientation

	hYaw = math.atan2(h[1][0], h[0][0])
	hPitch = -math.asin(h[2][0])
	#hRoll = math.atan2(m[2][1], m[2][2])

	cYaw = math.atan2(c[1][0], c[0][0])
	cPitch = -math.asin(c[2][0])

	if status == True:
		# Get the difference from the angles
		rYaw = hYaw - cYaw
		rPitch = hPitch - cPitch

		#print("B=%.4f | H= %.4f <= %.4f <= %.4f | %.4f" % (cYaw, (cYaw - MAX_HEAD_YAW), (hYaw), (cYaw + MAX_HEAD_YAW), leftRight))

		# Allow the head to rotate only within a certain limit
		#  with respect to the rest of the body
		if  (-MAX_HEAD_YAW) <= (rYaw) <= (MAX_HEAD_YAW):
			head.applyRotation( [0.0, 0.0, leftRight], True)
		elif (rYaw < -MAX_HEAD_YAW and leftRight < 0):
			head.applyRotation( [0.0, 0.0, leftRight], True)
		elif (rYaw > MAX_HEAD_YAW and leftRight > 0):
			head.applyRotation( [0.0, 0.0, leftRight], True)
		else:
			body.applyRotation( [0.0, 0.0, leftRight], True)
			head.applyRotation( [0.0, 0.0, 0.0], True)

		#print("B=%.4f | H= %.4f <= %.4f <= %.4f | %.4f" % (cPitch, (cPitch - MAX_HEAD_PITCH), (hPitch), (cPitch + MAX_HEAD_PITCH), upDown))

		# Allow the head to rotate up and down only within a certain limit
		#  with respect to the rest of the body
		if  (-MAX_HEAD_PITCH) <= (rPitch) <= (MAX_HEAD_PITCH):
			head.applyRotation( [0.0, upDown/10, 0.0], True)
		elif (rPitch < -MAX_HEAD_PITCH and leftRight < 0):
			head.applyRotation( [0.0, upDown, 0.0], True)
		elif (rPitch > MAX_HEAD_PITCH and leftRight > 0):
			head.applyRotation( [0.0, upDown, 0.0], True)
		# Don't really need to rotate the body in this direction
		else:
		#	body.applyRotation( [0.0, upDown, 0.0], True)
			head.applyRotation( [0.0, 0.0, 0.0], True)

		# Center mouse in game window
		Rasterizer.setMousePosition(width//2, height//2)


def mouse_move(human, mouse, width, height):
	""" Get the movement of the mouse as an X, Y coordinate. """
	# distance moved from screen center
	# Using the '//' operator (floor division) to produce an integer result
	x = width//2 - mouse.position[0]
	y = height//2 - mouse.position[1]

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
