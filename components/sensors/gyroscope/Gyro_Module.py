import sys, os
import GameLogic
import Mathutils


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

def init(contr):
	# Middleware initialization
	if not hasattr(GameLogic, 'orsConnector'):
		GameLogic.orsConnector = MiddlewareConnector()

	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)

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
		print ('######## GYROSCOPE INITIALIZATION ########')
		robot_state_dict['gyro_angle'] = 0.0
		print ("OPENING PORTS '{0}'".format(port_name))
		GameLogic.orsConnector.registerBufferedPortBottle([port_name])
		#GameLogic.orsConnector.printOpenPorts()
		print ('######## GYROSCOPE INITIALIZED ########')


def output(contr):
	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)

	if ob['Init_OK']:	
		robot_state_dict = GameLogic.robotDict[parent]

		############### Gyroscope ###################

		# Compute the angle with respect to the world
		rot_matrix = ob.worldOrientation
		local_X_vector = Mathutils.Vector(rot_matrix[0])

		# Ignore the Z direction (keep the angle restrained to the XY plane)
		local_X_vector[2] = 0
		local_X_vector.normalize()

		world_X_vector = Mathutils.Vector([1,0,0])
		world_Y_vector = Mathutils.Vector([0,-1,0])
		try:
			gyro_angle = Mathutils.AngleBetweenVecs(local_X_vector, world_X_vector)
			dot = local_X_vector.dot(world_Y_vector)
			if dot < 0:
				gyro_angle = gyro_angle * -1

			# Store the value in the robot's dictionary
			robot_state_dict['gyro_angle'] = gyro_angle
			# Store the value in the sensor component's properties
			#  (for display using Blender Debug)
			ob['Gyro_angle'] = gyro_angle
			#print ("Gyroscope angle: ", gyro_angle, " >> Dot: ", dot)

			if GameLogic.orsCommunicationEnabled:
				p = GameLogic.orsConnector.getPort(port_name)
				bottle = p.prepare()
				bottle.clear()
				bottle.addDouble(gyro_angle)
				#...and send it
				p.write()	

		except AttributeError as detail:
			print ("Can't determine angle")
			print (detail)


