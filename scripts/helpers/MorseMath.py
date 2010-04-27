import math
import GameLogic
from Mathutils import *

# euler_angle(object) returns a tuple yaw, pitch, roll, in degree,
# corresponding to the transformation between the origin frame and the object
# frame.
#
# It uses the built-in blender function toEuler
def euler_angle(ob):
	rot_matrix = ob.orientation
	matrix = Matrix(rot_matrix[0], rot_matrix[1], rot_matrix[2])

	# XXX If we want to have the right handle, we definivly need to transpose
	# the matrix here, but why ?
	matrix.transpose()
	# Use the correct function, depending on the version of Blender
	if GameLogic.pythonVersion >= 3:
		euler = matrix.to_euler()
	else:
		euler = matrix.toEuler()

	return [euler.z, euler.x, euler.y]

# euler_angle(object) returns a tuple yaw, pitch, roll, in degree,
# corresponding to the transformation between the origin frame and the object
# frame.
#
# It is computed "by hand", using the method described in 
#  http://planning.cs.uiuc.edu/node103.html
def euler_angle_old(ob):
	rot_matrix = ob.orientation

	alpha = math.atan2 (rot_matrix[1][0], rot_matrix[0][0])
	beta = math.atan2 (-rot_matrix[2][0], math.sqrt(math.pow(rot_matrix[2][1], 2) + math.pow(rot_matrix[2][2], 2)))
	gamma = math.atan2 (rot_matrix[2][1], rot_matrix[2][2])

	yaw = math.degrees(alpha)
	roll = math.degrees(beta)
	pitch = math.degrees(gamma)

	#### WARNING ####
	# This is probably a temporary solution, while Blender 2.5
	#  is made more stable. Currently it provides a rotation matrix
	#  with opposite signs to that of Blender 2.49b
	#
	# Change the signs of the angles
	#  if the Blender version is 2.5
	if GameLogic.pythonVersion >= 3:
		yaw = yaw * -1
		pitch = pitch * -1
		roll = roll * -1

	return [yaw, pitch, roll]

# print_matrix_33 prints a 3x3 @matrix on stdout
def print_matrix_33 (matrix):
	for row in matrix:
		line = "[%.4f %.4f %.4f]" % (row[0], row[1], row[2])
		print (line)
