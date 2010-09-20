import Rasterizer
import GameLogic
import GameKeys
import Blender
import math

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
	
	m = head.orientation
	roll = math.atan2(m[1][0], m[0][0]) * 180 / math.pi
	pitch = -math.asin(m[2][0]) * 180 / math.pi
	#yaw = math.atan2(m[2][1], m[2][2]) * 180 / math.pi
	#print [roll, pitch, yaw]

	if status == True:
		# set the values
		
		if abs(roll + leftRight) > MAX_HEAD_PAN :# or abs(roll + leftRight) < abs(roll):
			body.applyRotation( [0.0, 0.0, leftRight], True )
			
		elif abs(roll + leftRight) < MAX_HEAD_PAN or abs(roll + leftRight) < abs(roll):
			head.applyRotation( [0.0, 0.0, leftRight], True )
	
		if abs(pitch + upDown) < MAX_HEAD_TILT or abs(pitch + upDown) < abs(pitch):
			head.applyRotation( [0.0, upDown, 0.0], True )
			
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
