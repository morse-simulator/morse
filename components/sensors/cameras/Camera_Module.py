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
#from Convert import convert

structObject = ''
# Default size for an image of 512 * 512
Image_Size_X = 640
Image_Size_Y = 480
Image_Size = 4 * Image_Size_X * Image_Size_Y

# Background color for the captured images (Default is blue)
#bg_color = [0, 0, 255, 255]
# Gray
bg_color = [143,143,143,255]

def init(contr):
	global structObject
	global Image_Size_X
	global Image_Size_Y
	global Image_Size

	print ('######## CAMERA INITIALIZATION ########')

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
		print ("ERROR: Unable to create the port:")
		print (detail)

	# Create a key for a dictionary of cameras
	#  necesary if there are more than one camera added to the scene
	key = 'Camera'
	if GameLogic.pythonVersion < 3:
		screen_name = 'OBCameraCube'
		camera_name = 'OBCameraRobot'
	else:
		screen_name = 'CameraCube'
		camera_name = 'CameraRobot'
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
	print ("Camera: Key being used is: '{0}'".format(key))

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

	#### WARNING 2.5 ####
	# As of 19 / 03 / 2010 Blender 2.5 is crashing when trying
	#  to export images of size 512 X 512
	# Temporarily restricting the size to 256 X 256
	if GameLogic.pythonVersion > 3:
		Image_Size_X = 256
		Image_Size_Y = 256
		Image_Size = 4 * Image_Size_X * Image_Size_Y

	# Set the background to be used for the render
	GameLogic.tv[key].source.background = bg_color
	# Define an image size. It must be powers of two. Default 512 * 512
	GameLogic.tv[key].source.capsize = [Image_Size_X, Image_Size_Y]
	print ("Camera: Exporting an image of capsize: {0} pixels".format(GameLogic.tv[key].source.capsize))

	# Set a filter to produce images in grayscale
	# NOT YET IMPLEMENTED IN BLENDER.
	#GameLogic.tv[key].source.filter = Gray

	# Create an instance of the Struct object,
	# to make the unpacking of the captured images more efficient
	structObject = struct.Struct('=BBB')

	# Check that the conversion buffer could be initialized
	#if not convert.init_array(Image_Size):
		#ob['Init_OK'] = True
	ob['Init_OK'] = True

	#print_properties(ob)

	print ('######## CAMERA INITIALIZED ########')


def print_properties(ob):
	# Read the list of properties
	properties = ob.getPropertyNames()

	print ("Properties of object: " + ob)
	for prop in properties:
		print (prop + " = " + ob[prop])


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

				"""
				# USING THE C LIBRARY TO CONVERT THE IMAGE FORMAT
				# The SWIG binding extracts the length of the string
				info = convert.convert_image( image_string )
				GameLogic.orsConnector.postImageRGB(info, imX, imY, port_name)
				"""

				# Don't do any conversion, send the image as RGBA (yarp 2.2.5)
				data = array.array('B',image_string)
				info = data.buffer_info()
				GameLogic.orsConnector.postImageRGBA(info, imX, imY, port_name)

				"""
				# Data conversion in Python (OLD and SLOW)
				buf = decode_image (image_string)
				# Convert it to a form where we have access to a memory pointer
				data = array.array('B',buf)
				info = data.buffer_info()
				GameLogic.orsConnector.postImageRGB(info, imX, imY, port_name)
				"""
