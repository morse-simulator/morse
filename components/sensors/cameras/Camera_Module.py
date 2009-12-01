import sys, os
import GameLogic
import VideoTexture
import array, struct

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
import setup.ObjectData

#import ors_image_yarp
from Convert import convert

structObject = ''
# Default size for an image of 512 * 512
Image_Size_X = 512
Image_Size_Y = 512
Image_Size = 4 * Image_Size_X * Image_Size_Y

def init(contr):
	global structObject

	print '######## CAMERA INITIALIZATION ########'
	print

	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)

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
	key = 'Camera'
	screen_name = 'OBCameraCube'
	camera_name = 'OBCameraRobot'
	texture_name = 'IMplasma.png'
	name_len = len(ob.name)
	if name_len > 4 and ob.name.endswith('.00', name_len-4, name_len-1):
		extension = ob.name[name_len-4:]
		key = key + extension
		screen_name = screen_name + extension
		camera_name = camera_name + extension
		#texture_name = texture_name + extension
	# Store the key as an ID in the Empty object
	ob['camID'] = key
	print "KEY BEING USED IS: '{0}'".format(key)

	# Get the reference to the camera and screen
	scene = GameLogic.getCurrentScene()
	screen = scene.objects[screen_name]
	camera = scene.objects[camera_name]

	# Link the objects using VideoTexture	
	if not hasattr(GameLogic, 'tv'):
		GameLogic.tv = {}

	matID = VideoTexture.materialID(screen, texture_name)	
	GameLogic.tv[key] = VideoTexture.Texture(screen, matID)
	GameLogic.tv[key].source = VideoTexture.ImageRender(scene,camera)

	# Set the background to be used for the render
	#GameLogic.tv[key].source.background = [255,100,0,255]
	# Define an image size. It must be powers of two. Default 512 * 512
	GameLogic.tv[key].source.capsize = [Image_Size_X, Image_Size_Y]
	print "EXPORTING AN IMAGE OF CAPSIZE: ", GameLogic.tv[key].source.capsize

	# Create an instance of the Struct object,
	# to make the unpacking of the captured images more efficient
	structObject = struct.Struct('=BBB')

	if not convert.init_array(Image_Size):
		ob['Init_OK'] = True

	#print_properties(ob)

def print_properties(ob):
	# Read the list of properties
	properties = ob.getPropertyNames()

	print "Properties of object: ", ob
	for prop in properties:
		print prop, " = ", ob[prop]


def update(contr):
	ob = contr.owner

	# refresh video
	if hasattr(GameLogic, 'tv') and ob['Init_OK']:
		GameLogic.tv[ob['camID']].refresh(True)


# Conversion from a string to an array of integers
# From the blender image format to that of YARP
def decode_image (image_string):
	"""	Remove the alpha channel from the images taken from Blender.
		Convert the binary images to an array of integers, to be
		passed to the middleware.
		NOTE: Changing to an array of integers is not necessary
			Could possibly keep using a string. Need testing."""
	length = len(image_string)
	image_buffer = []
	k = 0

	# Grab 4 bytes of data, representing a single pixel
	for i in range(0, length, 4):
		rgb = structObject.unpack(image_string[i:i+3])
		image_buffer.extend ( rgb )

	"""
	print "PYTHON Converted string of length: ", length
	print "\tFrom: '{0}'".format(image_string)
	print "\tTo  : ", image_buffer
	"""

	return image_buffer


def grab(contr):
	""" Capture the image currently viewed by the camera.
		Convert the image and send it trough a port. """

	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)

	if ob['Init_OK']:
		# execute only when the 'grab_image' key is released
		# (if we don't test that, the code get executed two times,
		#	when pressed, and when released)
		sensor = GameLogic.getCurrentController().sensors['Check_capturing']

		if sensor.positive:
			# extract VideoTexture image
			if hasattr(GameLogic, 'tv'):
				imX,imY = GameLogic.tv[ob['camID']].source.size
				image_string = GameLogic.tv[ob['camID']].source.image

				# Data conversion in Python (OLD)
				#buf = decode_image (image_string)

				# TESTING THE C LIBRARY
				# The SWIG binding extracts the length of the string
				info = convert.convert_image( image_string )
				GameLogic.orsConnector.postImage(info, imX, imY, port_name)


				"""
				# TESTING THE C++ LIBRARY
				# nada = ors_image_yarp.convert_and_send_yarp( image_string, port_name )
				"""

				"""
				# Trying to avoid the conversion altogether
				data = array.array('B',image_string)
				info = data.buffer_info()
				GameLogic.orsConnector.postImage(info, imX, imY, port_name)
				"""

			"""
			# Convert it to a form where we have access to a memory pointer
			data = array.array('B',buf)
			info = data.buffer_info()

			GameLogic.orsConnector.postImage(info, imX, imY, port_name)
			"""
