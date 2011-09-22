# -*- coding: utf-8 -*-
'''
 The next steps will be proceed :
    - DtmObject: Create an heightmap (mesh) from the infile creating a plane
      with 1 vertice in the middle of each pixel. No texture is applied.
      Using:
        def __init__(self, infile):
        def readDTMHeader(self, infile):
        def drawDTM(self, fub):
    - Adding texture with setOrthoTexture. 
        def setOrthoTexture(self, dirname):
        def TextureMapTif(self, filename, coords):
        def UVMapTif(self, filename, coords):
    - Other utilities:
        def distance2D(self, A, B):
        def findZOfClosestPoint(self, findcoors):
'''

bl_addon_info = {
    'name': 'Digital Terrain Map Loader',
    'author': 'Redouane Boumghar (LAAS - Magellium)',
    'version': '2011/09/22',
    'blender': (2, 5, 9),
    'location': 'Mesh',
    'description': 'Plugin to load geographical data',
    'warning': '', # used for warning icon and text in addons panel
    'wiki_url': 'http://www.openrobots.org/wiki/morse/',
    'tracker_url': '',
    'category': 'Robotics'}

##### IMPORTS
#import osgeo # Gdal python bindings
#import shapelib
import os
import bpy
import logging
##### SUB-IMPORTS
#from bpy import *

class DtmObject:
    """A DTM Class able to read ASCII DTM IGN Files"""
    # Height 2 meters (image values are real altitude values): z scale factor:
    z2m = 1.0
    # How many meters per Blender Unit (1 Blender Unit = 1 meters)
    u2m = 1.0

    # Numbers
    NCols = 0
    NRows = 0
    meanZ = 0
    NODATA = -9999

    # Everything is in meters: UTM origins represents the position of the low left corner of the DTM
    UTMXOrigin = 0
    UTMYOrigin = 0
    UTMZOrigin = 0
    CellSize   = 0

    # Blender objects 
    dtm_mesh  = [] # mesh
    dtm_object = [] # scene object

    #------------------------------------------#
    def TextureMapTif(self, filename, coords):
        # Cropping image, the input image in filename is considering to totally overlap the DTM.
        dalMetricXSize = self.CellSize*self.NCols
        dalMetricYSize = self.CellSize*self.NRows
        # X offset is self.UTMXOrigin - Xmin_in_coords
        dalXOffset = self.UTMXOrigin - coords[0]
        dalYOffset = coords[3] - (self.UTMYOrigin + dalMetricYSize) 
        #logging.info('[geoDTMLoader] [cropping] ', dalMetricXSize/coords[6], ' x ', dalMetricYSize/coords[6], ' from point ', dalXOffset/coords[6], dalYOffset/coords[6])
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
        self.dtm_mesh.materials = [dtmmat] #.append(dtmmat)
        #self.dtm_mesh.link(dtmmat)
        self.dtm_mesh.update()
        #scene = bpy.Scene.GetCurrent()
        #self.dtm_object = scene.objects.new(self.dtm_mesh, 'morseGeoDTM')
        #self.dtm_object.link(self.dtm_mesh)
        #self.dtm_object.colbits = (1<<0) + (1<<5)
        bpy.Window.Redraw()
        
    #------------------------------------------#
    def UVMapTif(self, filename, coords):
        logging.info('[geoDTMLoader] UV MAPPING   '+filename)
        # Adding a spare material (this sets specularity and other properties)
        newmat = Material.New('newmat')
        self.dtm_mesh.materials.append(newmat)
        ## Example for UV textured faces selection:
        faces = self.dtm_mesh.faces
#
#        selected_faces = []
#        SEL = Mesh.FaceFlags['SELECT']
#        # get selected faces:
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
#            f.flag |= SEL
#            if f.flag & SEL:
#                selected_faces.append(f)


#        me.faces.append(f) 
#        f.transp=NMesh.FaceTranspModes['SOLID']
#        f.flag=NMesh.FaceFlags['ACTIVE']

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
                    logging.info(ishortname)
                    while L:
                        geoData[i] = float( ((L.split(':'))[1]) )
                        L = fub.readline()
                        i = i + 1
                    logging.info(geoData)
                else: 
                    logging.info('[geoDTMLoader] (WW) No georeference for this image :'+filename)
                # Calculating borders of the footlogging.info: fullGeoData
                if fullGeoData[0] < 0.0 and geoData[0] >= 0.0:
                    fullGeoData = geoData[:]
                else:
                    logging.info('[geoDTMLoader] Updating full')
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
        logging.info(fullGeoData)
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
        # self.UTMXOrigin and self.UTMYOrigin hold values in Lambert 93
        # They must be transformed into UTM : TODO
        #---


        #---
        # Feed the Scene_Script_Holder object with the georeferenced information
        # blender (0,0,z) correspond to real utm (UTMXOrigin, UTMYOrigin, z)
        # This information is saved in properties of the Scene_Script_Holder object.
        #---

        # Checking if the Scene_Script_Holder exists else no info saving.
        ooo = None
        try:
            ooo = bpy.data.objects['Scene_Script_Holder']
        except:
            ooo = None

        if ooo == None:
            # Warn that no georeferencing is possible
            logging.info('[geoDTMLoader] This scene can not be georeferenced: Scene_Script_Holder does not exist')
        else:
            '''
            # Check existence of properties UTMXOffset and UTMYOffset
            # Used to get real world georeferenced coordinates
            # z_blender = z_reality
            # x_reality = x_blender + UTMXOffset
            # idem for y 
            px = None
            try:
                px = ooo.get('UTMXOffset')
            except:
                # No property with that name
                px = None
            if px != None:
                bpy.data.objects['Scene_Script_Holder']['UTMXOffset'] = str(UTMXOrigin)

            try:
                py = ooo.get('UTMYOffset')
            except:
                # No property with that name
                py = None
            if py != None:
                bpy.data.objects['Scene_Script_Holder']['UTMYOffset'] = str(UTMYOrigin)
            '''
            bpy.data.objects['Scene_Script_Holder']['UTMXOffset'] = str(self.UTMXOrigin)
            bpy.data.objects['Scene_Script_Holder']['UTMYOffset'] = str(self.UTMYOrigin)

            

        #---
        # To provide continuous reading
        return fub


    #------------------------------------------#
    def __init__(self, infile):
        self.meanZ = 0
        self.dtm_mesh  = bpy.data.meshes.new('morseDTMmesh')  # DTM Mesh Object
        #--- Get the DTM data
        fub = self.readDTMHeader(infile)
        self.drawDTM(fub)

    def drawDTM(self, fub):
        #---
        # Positioning 3D points on the Digital Terrain Map
        #---
        logging.info('[geoDTMLoader] Loading 3D DTM points... ')

        #--- Registering low left corner as the global map origin = Blender (0,0,z)
        ysize = self.NRows*self.CellSize
        xsize = self.NCols*self.CellSize
        # The Y start is ysize as the data starts with the up left corner even
        # if the DTM is registrered via its low left corner
        # And DTM data represents the altitude of the middle of the cells.
        y = (ysize - self.CellSize/2.0)/self.u2m;
        x = (self.CellSize/2.0)/self.u2m;
        z = 0
        kv = 0 # vertex index
        myvertices2append = []
        logging.info('[geoDTMLoader] NRows = %d  |  NCols = %d ', self.NRows, self.NCols)
        for i in range(self.NCols):
            myvertices2append.append((0.0, 0.0, 0.0))
        myvertices = []
        for j in range(self.NRows):
            fubline = fub.readline()
            # Adding NCols points at each line/row: this is a way of optimizing
            #self.dtm_mesh.vertices.add(self.NCols)
            # Reading the line and updating each point coordinates
            kv = 0
            for i in fubline.split():
                z = float(i)
                self.meanZ = self.meanZ + z
                if z == self.NODATA:
                    z = 0
                myvertices2append[kv]=(x, y, z/self.z2m)
                # Going on next cell
                x = x + self.CellSize/self.u2m
                kv = kv + 1
            #--- Flat append of vertices
            logging.debug(myvertices2append)
            myvertices += myvertices2append
            #--- Starting a new line
            x = (self.CellSize/2.0)/self.u2m
            # y's are inverted since the DTM start is on the top left corner.
            y = y - self.CellSize/self.u2m
        fub.close()
        #--- calculating the Z mean for later camera placement or else. 
        self.meanZ = self.meanZ / (float(self.NRows)*float(self.NCols))
        #---

        #--- Filling Faces
        logging.info('[geoDTMLoader] Filling faces')
        v   = 0
        vmax = self.NCols*self.NRows - self.NCols
        #--- Add all facing one by one (certainly not the best solution)
        myfaces = []
        while v < vmax:
            if (((v+1)%self.NCols) != 0): 
                myfaces.append((v, v+1, v+self.NCols+1, v+self.NCols))
            v = v + 1
        #--- Final update 
        logging.info('[geoDTMLoader] Creating and linking the mesh object')
        self.dtm_object = bpy.data.objects.new("morseGeoDTM", self.dtm_mesh) 
        bpy.context.scene.objects.link(self.dtm_object)

        logging.info('[geoDTMLoader] Updating the mesh')
        #bpy.data.scenes[0].objects.link(self.dtm_object)
        self.dtm_mesh.from_pydata( myvertices, [], myfaces)
        self.dtm_mesh.update(calc_edges=True)
        #---
        logging.info('[geoDTMLoader] Drawing DTM done.')

    #------------------------------------------#
    def distance2D(self, A, B):
        x = B[0] - A[0]
        y = B[1] - A[1]
        return sqrt((x*x + y*y))

    #------------------------------------------#
    def findZOfClosestPoint(self, findcoors):
        smallestDist = -1
        smallestZ    = 0
        for v in self.dtm_mesh.vertices:
            if smallestDist == -1: # smallestDist init.
                smallestDist = self.distance2D(findcoors, v.co)
                smallestZ    = v.co[2]
            else:
                tmp = self.distance2D(findcoors, v.co)
                if smallestDist > tmp:
                    smallestDist = tmp
                    smallestZ    = v.co[2]
        return smallestZ




