import sys, os
import bpy
from Blender import Draw, BGL, Image


try:
	scriptRoot = os.path.join(os.environ['ORS_ROOT'],'scripts')
except KeyError:
	scriptRoot = '.'

if scriptRoot not in sys.path:
	sys.path.append(scriptRoot)

from helpers import ors_properties

menu = Draw.Create(0)
mymsg = ""
toggle = 0
#ors_image = Image.Load("../../../Textures/openrobots-simulator-icon_big.png")

def event(evt, val):		# the function to handle input events
	global mymsg

	"""
	if not val:	# val = 0: it's a key/mbutton release
		if evt in [Draw.LEFTMOUSE, Draw.MIDDLEMOUSE, Draw.RIGHTMOUSE]:
			mymsg = "You released a mouse button."
			Draw.Redraw(1)
		return
	"""

	if evt == Draw.ESCKEY:
		Draw.Exit()								 # exit when user presses ESC
		return

	#else: return # no need to redraw if nothing changed

	Draw.Redraw(1)

def button_event(evt):	# the function to handle Draw Button events
	global mymsg, toggle
	if evt == 1:
		mymsg = "You pressed the toggle button."
		toggle = 1 - toggle
		Draw.Redraw(1)
	if evt == 2:
		mymsg = "User selected the menu option {0}".format(menu.val)
		ob = bpy.data.objects["CameraMain"]
		ors_properties.write_property (ob, "Operation_Mode", menu.val)
		Draw.Redraw(1)


def gui():							# the function to draw the screen
	global mymsg, toggle, menu

	"""
	# Only needed for alpha blending images with background.
	BGL.glEnable( BGL.GL_BLEND )
	BGL.glBlendFunc(BGL.GL_SRC_ALPHA, BGL.GL_ONE_MINUS_SRC_ALPHA) 
	Draw.Image(ors_image, 60, 50)
	BGL.glDisable( BGL.GL_BLEND )
	"""



	BGL.glClearColor(0,0,1,1)
	BGL.glClear(BGL.GL_COLOR_BUFFER_BIT)
	BGL.glColor3f(1,1,1)

	"""
	Draw.Toggle("Toggle", 1, 10, 10, 55, 20, toggle,"A toggle button")
	BGL.glRasterPos2i(72, 16)
	if toggle: toggle_state = "down"
	else: toggle_state = "up"
	Draw.Text("The toggle button is %s." % toggle_state, "small")
	BGL.glRasterPos2i(10, 230)
	BGL.glColor3f(1,0.4,0.3)
	"""

	BGL.glRasterPos2i(50, 260)
	Draw.Text("Open Robots Simulator", "large")
	BGL.glRasterPos2i(50, 200)
	Draw.Text("Camera Module", "large")


	BGL.glRasterPos2i(50, 140)
	Draw.Text("Select camera type:")

	BGL.glColor3f(1,0.4,0.3)
	BGL.glRasterPos2i(50, 60)
	Draw.Text(mymsg)


	#--------------------------#
	#      IMPORTANT NOTE      #
	#--------------------------#
	# The number after each entry in the menu should be a binary mask
	#  for the Game Engine States that will be activated
	name = "Camera Mode %t|Image %x1|Cognitive %x2|Image & Cognitive %x3|Other %x4"
	# The 'menu.val' in this case may not match the selected item,
	# Since it refers to the index of the list of options
	menu = Draw.Menu(name, 2, 50, 100, 150, 20, menu.val, "Camera output mode.")



Draw.Register(gui, event, button_event)	# registering the 3 callbacks
