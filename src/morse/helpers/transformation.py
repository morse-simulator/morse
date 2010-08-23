import math
import GameLogic
if GameLogic.pythonVersion < 3:
	import Mathutils as mathutils
else:
	import mathutils
#from Mathutils import *

class Transformation3d:
	def __init__(self, ob):
		self.matrix = mathutils.Matrix([1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1])
		self.euler = mathutils.Euler([0, 0, 0])
		if ob != None:
			self.update(ob)

	@property
	def x(self):
		return self.matrix[0][3]

	@property
	def y(self):
		return self.matrix[1][3]

	@property
	def z(self):
		return self.matrix[2][3]

	@property
	def yaw(self):
		return self.euler.z

	@property
	def pitch(self):
		return self.euler.x

	@property
	def roll(self):
		return self.euler.y

	def transformation3dWith(self, t3d):
		res = Transformation3d(None)
		o2m = self.matrix.copy()
		o2m.invert()
		res.matrix = o2m * t3d.matrix
		res.euler = res.matrix.toEuler()
		return res

	def update(self, ob):
		rot_matrix = ob.orientation
		self.matrix = mathutils.Matrix(rot_matrix[0], rot_matrix[1], rot_matrix[2])
		# XXX It seems incorrect to transpose the matrix here, but in other
		# context, we need to. It is very strange. Need more investigation
		self.matrix.resize4x4()

		pos = ob.position
		for i in range(0,3):
			self.matrix[i][3] = pos[i]
		self.matrix[3][3] = 1

		if GameLogic.pythonVersion < 3:
			self.euler = self.matrix.toEuler()
		else:
			self.euler = self.matrix.to_euler()

		

	def __str__(self):
		res = "x : " + str(self.x) + " y : " + str(self.y) + " z : " + str(self.z)
		res += " yaw : " + str(self.yaw) + " pitch : " + str(self.pitch) 
		res += " roll : " + str(self.roll)
		return res
