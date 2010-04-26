import sys, os
import GameLogic
import VideoTexture
import array
import math

import time
from Camera_Poster import ors_viam_poster
#from Convert import convert
from datetime import datetime;
from helpers.MorseTransformation import Transformation3d

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

# Background color for the captured images (Default is blue)
#bg_color = [0, 0, 255, 255]
# Gray
bg_color = [143,143,143,255]

def init(contr):
	global Image_Size_X
	global Image_Size_Y
	global Image_Size
	global Image_focal

	print ('######## CAMERA BASE INITIALIZATION ########')

	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)
	robot_state_dict = GameLogic.robotDict[parent]
	local_dict = GameLogic.componentDict[ob]
	
	# Default size for an image of 512 * 512
	#Image_Size_X = 320
	Image_Size_X = ob['cam_width']
	#Image_Size_Y = 240
	Image_Size_Y = ob['cam_height']
	#Image_focal = 25
	Image_focal = ob['cam_focal']

	# Middleware initialization
	if not hasattr(GameLogic, 'orsConnector'):
		GameLogic.orsConnector = MiddlewareConnector()
		
	# Create YARP Connection port
	try:
		GameLogic.orsConnector.registerPort([port_name])
	except NotImplementedError as detail:
		print ("ERROR: Unable to create the port:")
		print (detail)

	### POCOLIBS ###
	# Start the external poster module
	# poster_name = "morse_" + ob['Component_Type'] + "_poster"
	# poster_name = poster_name.upper()
	poster_name = "viamMorseBench"

	camera_list = []
	cameras = []
	# Create a list of the cameras attached to this component
	for child in ob.children:
		try:
			child['Component_Type']
			camera_object = child
			print ("Camera Base: Camera found with id: '{0}'".format(camera_object))
			camera_list.append(camera_object)

			#cameras = ors_viam_poster.imageInitArray(1)
			image_init = ors_viam_poster.simu_image_init()
			image_init.camera_name = camera_object.name
			image_init.width = Image_Size_X
			image_init.height = Image_Size_Y
			image_init.focal = Image_focal
			cameras.append(image_init)

		except KeyError:
			pass

	local_dict['camera_list'] = camera_list

	#Nb_image = ob['Num_Cameras']
	Nb_image = len(camera_list)
	baseline = 0
	if Nb_image == 2:
		pos_cam0 = camera_list[0].position
		pos_cam1 = camera_list[1].position
		baseline = math.sqrt( math.pow(pos_cam0[0] - pos_cam1[0], 2) +
							  math.pow(pos_cam0[1] - pos_cam1[1], 2) +	
							  math.pow(pos_cam0[2] - pos_cam1[2], 2)) 

	print ("Camera Base: Number of cameras found: {0}".format(Nb_image))
	robot_state_dict[port_name] = ors_viam_poster.init_data(poster_name, "stereo_bank", Nb_image, baseline, cameras[0], cameras[1])
	print ("Poster ID generated: {0}".format(robot_state_dict[port_name]))
	if robot_state_dict[port_name] == None:
		print ("ERROR creating poster. This module may not work")
		ob['Init_OK'] = False
	else:
		ob['Init_OK'] = True

	print ('######## CAMERA BASE INITIALIZED ########')



def main(contr):
	""" Capture the image currently viewed by the camera.
		Convert the image and send it trough a port. """
	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)
	robot_state_dict = GameLogic.robotDict[parent]
	local_dict = GameLogic.componentDict[ob]

	camera_list = local_dict['camera_list']

	if ob['Init_OK']:
		"""
		# execute only when the 'grab_image' key is released
		# (if we don't test that, the code get executed two times,
		#	when pressed, and when released)
		sensor = GameLogic.getCurrentController().sensors['Check_capturing']

		if sensor.positive:
		"""
		# extract VideoTexture image
		if hasattr(GameLogic, 'tv'):

			#Nb_image = ob['Num_Cameras']
			Nb_image = len(camera_list)

			### POCOLIBS ###
			mainToOrigin = Transformation3d(parent)

			pom_robot_position =  ors_viam_poster.pom_position()
			pom_robot_position.x = mainToOrigin.x
			pom_robot_position.y = mainToOrigin.y
			pom_robot_position.z = mainToOrigin.z
			pom_robot_position.yaw = robot_state_dict['Yaw']
			pom_robot_position.pitch = robot_state_dict['Pitch']
			pom_robot_position.roll = robot_state_dict['Roll']

			# Compute the current time ( we only requiere that the pom date
			# increases using a constant step so real time is ok)
			t = datetime.now()
			pom_date = int(t.hour * 3600* 1000 + t.minute * 60 * 1000 + 
					  t.second * 1000 + t.microsecond / 1000)

			ors_cameras = []
			ors_images = []

			# Cycle throught the cameras on the base
			# In normal circumstances, there will be two for stereo
			for ors_camera_id in camera_list:
				sensorToOrigin = Transformation3d(ors_camera_id)
				mainToSensor = mainToOrigin.transformation3dWith(sensorToOrigin)

				imX,imY = GameLogic.tv[ors_camera_id['camID']].source.size
				image_string = GameLogic.tv[ors_camera_id['camID']].source.image

				# Fill in the structure with the image information
				camera_data = ors_viam_poster.simu_image()
				camera_data.width = imX
				camera_data.height = imY
				camera_data.pom_tag = pom_date
				camera_data.tacq_sec = t.second
				camera_data.tacq_usec = t.microsecond
				camera_data.sensor = ors_viam_poster.pom_position()
				camera_data.sensor.x = mainToSensor.x
				camera_data.sensor.y = mainToSensor.y
				camera_data.sensor.z = mainToSensor.z
				# XXX +PI rotation is needed but I don't have any idea why !!
				camera_data.sensor.yaw = mainToSensor.yaw + 180.0
				camera_data.sensor.pitch = mainToSensor.pitch 
				camera_data.sensor.roll = mainToSensor.roll  

				ors_cameras.append(camera_data)
				ors_images.append(image_string)

			# Create the poster with the data for both images
			posted = ors_viam_poster.post_viam_poster(robot_state_dict[port_name], pom_robot_position, Nb_image, ors_cameras[0], ors_images[0], ors_cameras[1], ors_images[1])



def finish(contr):
	""" Procedures to kill the module when the program exits.
		12 / 04 / 2010
		Done for testing the closing of the poster. """

	ob, parent, port_name = setup.ObjectData.get_object_data(contr)
	robot_state_dict = GameLogic.robotDict[parent]

	print ("Component: {0} => Closing poster with id: {1}".format(ob, robot_state_dict[port_name]))
	ors_viam_poster.finalize(robot_state_dict[port_name])
	# Set the variable so that further calls to the main function will exit
	ob['Init_OK'] = False
	print ("Done!")
