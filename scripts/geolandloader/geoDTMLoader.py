#####################################################################################
#
# This script uses a blender file containing :
#  - Two cameras named : targetcam, obscam
#  - A lamp parent of the obscam. This lamp is a spot named obsspot to be
#    oriented to the observationnal area that the targetcam follow.
#  - An empty placed in the observationnal area that constrains the targetcam.
#  - An empty mesh to be used to create the heightmaps
#
#
# The next steps will be proceed :
#   1- Create an heightmap from the infile creating a plane with 1 vertice by pixel
#   2- Place the observed empty in the middle of the observed zone.
#   3- Place the lamp in the observation zone and render each placement.
#
#####################################################################################

##### IMPORTS
#import osgeo # Gdal python bindings
#import shapelib
import os
import Blender
##### SUB-IMPORTS
from Blender import Mesh, NMesh

class DtmObject:
	"""A DTM Class able to read ASCII DTM IGN Files"""
	# Height 2 meters (image values are real altitude values): z scale factor:
	z2m = 1.0
	# How many meters per Blender Unit (1 Blender Unit = 1 meters)
	u2m = 1.0

	# Other parameters
	verbose = 2

	# Numbers
	NCols = 0
	NRows = 0
	meanZ = 0
	NODATA = -9999

	# Everything is in meters: UTM origins represents the position of the low left corner of the DTM
	UTMXOrigin = 0
	UTMYOrigin = 0
	CellSize   = 0

	# Blender objects 
	DTMO  = [] # mesh
	DTMOO = [] # scene object

	#------------------------------------------#
	def TextureMapTif(self, filename, coords):
		# Cropping image, the input image in filename is considering to totally overlap the DTM.
		dalMetricXSize = self.CellSize*self.NCols
		dalMetricYSize = self.CellSize*self.NRows
		# X offset is self.UTMXOrigin - Xmin_in_coords
		dalXOffset = self.UTMXOrigin - coords[0]
		dalYOffset = coords[3] - (self.UTMYOrigin + dalMetricYSize) 
		print 'CROP INFO: ', dalMetricXSize/coords[6], ' x ', dalMetricYSize/coords[6], ' from point ', dalXOffset/coords[6], dalYOffset/coords[6]
		# HACK Considered as already CROPED
		#dataset = gdal.Open( filename, GA_ReadOnly )
		dtmtex = Texture.New('foo') 
		dtmtex.setType('Image')  
		img = Image.Load(filename) 
		dtmtex.image = img

		# MUST leave edit mode before changing an active mesh:
		in_editmode = Window.EditMode()
		if in_editmode: Window.EditMode(0)

		# Adding a spare material (this sets specularity and other properties)
		dtmmat = Material.New('dtmMat')
		dtmmat.setName('dtmMat')
		dtmmat.setTexture(0, dtmtex)
		dtmmat.emit = 0.5
		self.DTMO.materials = [dtmmat] #.append(dtmmat)
		#self.DTMO.link(dtmmat)
		self.DTMO.update()
		scene = Blender.Scene.GetCurrent()
		self.DTMOO = scene.objects.new(self.DTMO, 'morseDTMObject')
		#self.DTMOO.link(self.DTMO)
		#self.DTMOO.colbits = (1<<0) + (1<<5)
		Blender.Window.Redraw()
		
	#------------------------------------------#
	def UVMapTif(self, filename, coords):
		print 'UV MAPPING   '+filename
		# Adding a spare material (this sets specularity and other properties)
		newmat = Material.New('newmat')
		self.DTMO.materials.append(newmat)
		## Example for UV textured faces selection:
		faces = self.DTMO.faces
#
#		selected_faces = []
#		SEL = Mesh.FaceFlags['SELECT']
#		# get selected faces:
		# Building a UVmap 
		uv = [(0,0), (1,0), (1,1), (0,1)]
		for f in faces:
			f.uv = uv
			im=Image.Load('/tmp/test.tif')
			f.image = im 
			C = NMesh.FaceModes 
			f.mode=C['LIGHT']
			f.mode|=C['TEX']
			f.mode|=C['TWOSIDE']
#			f.flag |= SEL
#			if f.flag & SEL:
#				selected_faces.append(f)


#		me.faces.append(f) 
#		f.transp=NMesh.FaceTranspModes['SOLID']
#		f.flag=NMesh.FaceFlags['ACTIVE']

	#------------------------------------------#
	def setOrthoTexture(self, dirname):
		# List all tif files in the directory
		listing = os.listdir(dirname)
		geoData = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		fullGeoData = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		for filename in listing:
			(ishortname, ifileext) = os.path.splitext(filename)
			if ifileext == ".tif":
				# We want to texture the DTM
				# So we check for each available tif its overlapping with this DTM
				# Three different ways :
				#    - There is a shapefile for this tif: so we use it and UV Map it as specified by the shpfile.
				#    - There is no shapefile and we use the grf file so only a rectangle UV Mapping is done.
				#    - Our way: we mosaic all available tifs
				tabfile = os.path.join(dirname, ishortname+".tab")
				grffile = os.path.join(dirname, ishortname+".grf")
				#----- If Shape file exist then executing first alternative
				if os.path.isfile(tabfile):
					fub = open(grffile, 'r')
					L=fub.readline() # reading the file name.
					# geoData will hold the geo information concerning the ishortname+".tif" image
					# [0] -> X minimum
					# [1] -> X maximum
					# [2] -> Y minimum
					# [3] -> Y maximum
					# [4] -> X Center
					# [5] -> Y Center
					# [6] -> Pixel size on the terrain
					# [7] -> Number of lines
					# [8] -> Number of Columns
					L = fub.readline() # reading the first parameter
					i = 0
					print ishortname
					while L:
						geoData[i] = float( ((L.split(':'))[1]) )
						L = fub.readline()
						i = i + 1
					print geoData
				else: 
					print '(WW) No georeference for this image :'+filename
				# Calculating borders of the footprint: fullGeoData
				if fullGeoData[0] < 0.0 and geoData[0] >= 0.0:
					fullGeoData = geoData[:]
				else:
					print 'Updating full'
					# X minimum
					if geoData[0] < fullGeoData[0]:
						fullGeoData[0] = geoData[0]
					# X maximum
					if geoData[1] > fullGeoData[1]:
						fullGeoData[1] = geoData[1]
					# Y minimum
					if geoData[2] < fullGeoData[2]:
						fullGeoData[2] = geoData[2]
					# Y maximum
					if geoData[3] > fullGeoData[3]:
						fullGeoData[3] = geoData[3]
		# Deduct Center point
		fullGeoData[4] = (fullGeoData[0]+fullGeoData[1])/2.0
		fullGeoData[5] = (fullGeoData[2]+fullGeoData[3])/2.0
		# Deduct Number of pixels (assuming all images have the same resolution
		if fullGeoData[6] > 0.0:
			fullGeoData[7] = (fullGeoData[1]-fullGeoData[0])/fullGeoData[6]
			fullGeoData[8] = (fullGeoData[3]-fullGeoData[2])/fullGeoData[6]
		print fullGeoData
		self.TextureMapTif(os.path.join(dirname, 'mosaic.tif'), fullGeoData)


	#------------------------------------------#
	def readDTMHeader(self, infile):
		#--- Get the parameters from the header of the DTM file
		fub = open(infile, 'r')
		# Number of columns 
		fubline = fub.readline()
		self.NCols = int((fubline.split())[1])
		# Number of rows
		fubline = fub.readline()
		self.NRows = int((fubline.split())[1])
		# UTM origin X Low Left corner of the DTM
		fubline = fub.readline()
		self.UTMXOrigin = float((fubline.split())[1])
		# UTM origin Y Low Left corner of the DTM
		fubline = fub.readline()
		self.UTMYOrigin = float((fubline.split())[1])
		# Cells size
		fubline = fub.readline()
		self.CellSize = float((fubline.split())[1])
		# NODATA value
		fubline = fub.readline()
		self.NODATA = float((fubline.split())[1])
		#---
		# To provide continuous reading
		return fub


	#------------------------------------------#
	def __init__(self, infile):
		self.meanZ = 0
		self.DTMO  = Blender.Mesh.New('morseDTMmesh')  # DTM Mesh Object
		#--- Get the DTM data
		fub = self.readDTMHeader(infile)
		#---
		# Positioning 3D points on the Digital Terrain Map
		#---
		if self.verbose > 1:
			print 'Loading 3D DTM points... '
		#--- Registering low left corner as the global map origin = Blender (0,0,z)
		ysize = self.NRows*self.CellSize
		xsize = self.NCols*self.CellSize
		# The Y start is ysize as the data starts with the up left corner even
		# if the DTM is registrered via its low left corner
		# And DTM data represents the altitue of the middle of the cells.
		y = (ysize - self.CellSize/2.0)/self.u2m;
		x = (self.CellSize/2.0)/self.u2m;
		z = 0
		for j in range(self.NRows):
			fubline = fub.readline()
			for i in fubline.split():
				z = float(i)
				self.meanZ = self.meanZ + z
				if z == self.NODATA:
					z = 0
				# Directly Appending vertice to DTM Object self.DTMO
				self.DTMO.verts.extend(x,y,z/self.z2m)
				# Going on next cell
				x = x + self.CellSize/self.u2m
			# Starting a new line
			x = (self.CellSize/2.0)/self.u2m;
			# y's are inverted since the DTM start is on the top left corner.
			y = y - self.CellSize/self.u2m
		fub.close()
		# Updating variables
		self.DTMO.update()
		self.meanZ = self.meanZ / (float(self.NRows)*float(self.NCols))

		#---
		# Filling Faces
		#---
		if self.verbose > 1:
			print 'Filling faces ... '
		#---
		v   = 0
		n   = 0
		fff = []
		vmax = self.NCols*self.NRows - self.NCols
		#---
		while v < vmax:
			if (n < (self.NCols-1)): 
				# This if avoids making faces between last vertices and first
				# vertices, that would wrap the mesh.
				ff = NMesh.Face([self.DTMO.verts[v], self.DTMO.verts[v+1], self.DTMO.verts[v+self.NCols+1], self.DTMO.verts[v+self.NCols]])
				fff.append(ff)
			else:
				# Extending faces by line is faster than doing it one by one.
				self.DTMO.faces.extend(fff)
				fff[:] = []
				n = -1
			v = v + 1
			n = n + 1
		#---
		# Final update 
		self.DTMO.update()
		#---
		if self.verbose > 1:
			print ' done.'


