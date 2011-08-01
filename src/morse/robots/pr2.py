import GameLogic
import morse.core.robot
from morse.core.services import service
from morse.core.services import async_service
from morse.core import status


def find_dof_string(channel):
	"""
	Method that finds and returns the degree of freedom (dof) corresponding to the given channel.
	The dof has to be a blender_ik_setting.
	Each channel (bone) in the PR2 has just one dof.
	Returns the string of the axis corresponding to the dof.

	It's used as a helper method for this class.
	"""
	dof = 'x'
	if channel.ik_dof_y:
		dof = 'y'
	elif channel.ik_dof_z:
		dof = 'z'
	return dof


def find_dof_index(channel):
	"""
	Method that finds and returns the degree of freedom (dof) corresponding to the given channel.
	The dof has to be a blender_ik_setting.
	Each channel (bone) in the PR2 has just one dof.
	Returns the index of the axis as it is used in channel.joint_rotation

	It's used as a helper method for this class.
	"""
	dof = 0 # x
	if channel.ik_dof_y:
		dof = 1 # y
	elif channel.ik_dof_z:
		dof = 2 # z
	return dof


class PR2Class(morse.core.robot.MorseRobotClass):
	""" 
	Class definition for the PR2.
	Sub class of Morse_Object.

	This class has many MORSE Services that you can access via sockets/telnet
	"""

	def __init__(self, obj, parent=None):
		""" Constructor method.
			Receives the reference to the Blender object.
			Optionally it gets the name of the object's parent,
			but that information is not currently used for a robot. """
		# Call the constructor of the parent class
		print ("######## ROBOT '%s' INITIALIZING ########" % obj.name)
		super(self.__class__,self).__init__(obj, parent)

		print ('######## ROBOT INITIALIZED ########')

	
	@service
	def get_armatures(self):
		"""
		MORSE Service that returns a list of all the armatures on the PR2 robot.
		"""
		armatures = []
		# Search armatures in all objects parented to the pr2 empty
		for obj in self.blender_obj.childrenRecursive:
			# Check if obj is an armature
			if type(obj).__name__ == 'BL_ArmatureObject':
				armatures.append(obj.name)
		# Return status of service and list of armatures
		return(status.SUCCESS, armatures)


	@service
	def get_channels(self, armature_name):
		"""
		MORSE Service that returns a list of the channels (bones) in the given armature 'armature_name'
		"""
		armature = self.blender_obj.childrenRecursive[str(armature_name)]
		channels = []
		# add the name of each channel to the list
		for channel in armature.channels:
			channels.append(channel.name)
		return(status.SUCCESS, channels)

	
	@service
	def get_dofs(self, armature_name):
		"""
		MORSE Service that returns a dictionary with keys the channels of the given armature 'armature_name' 
		and as values the corresponding dof axis as 'x', 'y' or 'z'.
		"""
		armature = self.blender_obj.childrenRecursive[str(armature_name)]
		dofs = {}
		# find the dof of each channel
		for channel in armature.channels:
			dofs[channel.name] = find_dof_string(channel)
		return(status.SUCCESS, dofs)


	@service
	def get_dof(self, armature_name, channel_name):
		"""
		MORSE Service that returns the dof axis of the given channel 'channel_name' on the given armature 'armature_name'.
		"""
		armature = self.blender_obj.childrenRecursive[str(armature_name)]
		channel = armature.channels[str(channel_name)]
		dof = find_dof_string(channel)
		return(status.SUCCESS, dof)


	@service
	def get_rotations(self, armature_name):
		"""
		MORSE Service that returns a dict with keys the channel names of the given armature 'armature_name',
		and values the rotation xyz values.
		"""
		armature = self.blender_obj.childrenRecursive[str(armature_name)]
		rotations = {}
		# get the rotation of each channel
		for channel in armature.channels:
			rotations[channel.name] = channel.joint_rotation.to_tuple()
		return(status.SUCCESS, rotations)


	@service
	def get_rotation(self, armature_name, channel_name):
		"""
		MORSE Service that returns the rotation angles corresponding to the given channel 'channel_name' on the given armature 'armature_name'.
		"""
		armature = self.blender_obj.childrenRecursive[str(armature_name)]
		channel = armature.channels[str(channel_name)]
		# get the rotation in xyz
		rotation = channel.joint_rotation.to_tuple()
		return(status.SUCCESS, rotation)


	@service
	def set_dof_rotations(self, armature_name, angles):
		"""
		MORSE Service to set the rotion angles of the corresponding dof axes of the channels in 'armature_name'.
		angles must be a dict with as keys all the channel names of the given armature. And as values
		the angle of the channel dof axis that you want to set.
		"""
		angles = eval(angles)
		armature = self.blender_obj.childrenRecursive[str(armature_name)]
		# set the rotation of each channel
		for channel in armature.channels:
			print("Channel: ", channel.name)
			dof = find_dof_index(channel)
			#print("channel rotation before: " + str(channel.joint_rotation))
			rotation = channel.joint_rotation
			rotation[dof] = angles[channel.name]
			print("requested rotation: " + str(angles[channel.name]))
			channel.joint_rotation = rotation
			armature.update()
		
		return(status.SUCCESS, None)


	@service
	def set_dof_rotation(self, armature_name, channel_name, angle):
		"""
		MORSE Service to set the rotation angle of the corresponding dof axis of the 'channel_name' in 'armature_name'.
		"""
		armature = self.blender_obj.childrenRecursive[str(armature_name)]
		channel = armature.channels[str(channel_name)]
		dof = find_dof_index(channel)
		#print("channel rotation before: " + str(channel.joint_rotation))
		rotation = channel.joint_rotation
		rotation[dof] = float(angle)
		print("requested rotation: " + str(rotation))
		channel.joint_rotation = rotation
		armature.update()
		#print("channel rotation after: " + str(channel.joint_rotation))
		return(status.SUCCESS, None)


	def default_action(self):
		""" Main function of this component. """
		pass
