import shapelib, dbflib
import Blender
from Blender import *
import math
from math import sqrt
#
#       The the shapefile module
#

# SHAPELIB Object Types 
#----------------------------------------------
#define SHPT_NULL             0
#----------------------------------------------
#2D Shape Types (pre ArcView 3.x):
#
#define SHPT_POINT        1    Points
#define SHPT_ARC        3    Arcs (Polylines, possible in parts)
#define SHPT_POLYGON        5    Polygons (possible in parts)
#define SHPT_MULTIPOINT    8    MultiPoint (related points)

#----------------------------------------------
# 3D Shape Types (may include "measure" values for vertices):
#
#define SHPT_POINTZ        11    
#define SHPT_ARCZ        13
#define SHPT_POLYGONZ        15
#define SHPT_MULTIPOINTZ     18

#----------------------------------------------
# 2D + Measure Types:
#
#define SHPT_POINTM        21
#define SHPT_ARCM        23
#define SHPT_POLYGONM        25
#define SHPT_MULTIPOINTM     28
#----------------------------------------------
# Complex (TIN-like) with Z, and Measure:
#
#define SHPT_MULTIPATCH     31
#----------------------------------------------

# --------------------------- def read_shapefile(filename):
# open the shapefile
#shp = shapelib.ShapeFile(filename)

# the info method returns a tuple (num_shapes, type, min, max) where
# num_shapes is the number of shapes, type is the type code (one of
# the SHPT* constants defined in the shapelib module) and min and
# max are 4-element lists with the min. and max. values of the
# vertices.
#logging.info(shp.info())

# read_object reads a shape
#obj = shp.read_object(0)

# The vertices method returns the shape as a list of lists of tuples.
#logging.info(obj.vertices()[0][:10])

# The extents returns a tuple with two 4-element lists with the min.
# and max. values of the vertices.
#logging.info(obj.extents())

# The type attribute is the type code (one of the SHPT* constants
# defined in the shapelib module)
#logging.info(obj.type)

# The id attribute is the shape id
#logging.info(obj.id)

# the cobject method returns a PyCObject containing the shapelib
# SHPHandle. This is useful for passing shapefile objects to
# C-Python extensions.
#logging.info(shp.cobject())
# --------------------------- end of def read_shapefile(filename):


#--------------------------------------------------#
def distance2D(A, B):
    x = B.co[0] - A.co[0]
    y = B.co[1] - A.co[1]
    return sqrt((x*x + y*y))

#--------------------------------------------------#
def LoadBuildings(shp, hostObject):
    # Default parameters for ground altitude and building height
    BAlt       = 321
    BAltOffset = -5
    BHeight    = 12
    RoofHeight = 15
    # Preparing Texture
    buildingTex = Texture.New('buildingTex')
    buildingTex.setType('Image')
    img = Image.Load('/tmp/building.jpg')
    buildingTex.image = img
    # Default Material used for Buildings
    buildingMat = Material.New('buildingMat')
    buildingMat.rgbCol = [0.78,0.75, 0.4]
    buildingMat.emit = 0.3
    buildingMat.setSpec(0.0)
    buildingMat.setTexture(0, buildingTex)

    # Building barracks 
    Nshapes = (shp.info())[0]
    logging.info('Loading ', Nshapes, ' BUildings')
    for i in range(Nshapes):
        # Oshp is a shape object
        Oshp = shp.read_object(i)
        #for v in Oshp.vertices()[0]
        #--- Building the building with the shapefile's vertices
        groundCoverage = Oshp.vertices()[0]
        N = len(groundCoverage)
        logging.info('Building ', (i+1), 'with ', N, ' vertices')
        buildingMesh  = Blender.Mesh.New('buildingMesh');
        #--- Average the altitude of building first floor
        if (N > 0):
            thisBuildingZ = hostObject.findZOfClosestPoint(groundCoverage[0]) - BAltOffset
        else:
            thisBuildingZ = hostObject.meanZ - BAltOffset
        for i in range(N-1):
            # Get the Z of the closest vertices in DTM to adjust BZ
            # for j = TODO
            # Extend the vertices of the current building's mesh
            buildingMesh.verts.extend(groundCoverage[i][0]   - hostObject.UTMXOrigin,
                                        groundCoverage[i][1] - hostObject.UTMYOrigin ,
                                        thisBuildingZ)
        #--- Filling the face of the building's ground
        if (N == 5) or (N == 4):
            ff = NMesh.Face([buildingMesh.verts]);
            buildingMesh.faces.extend(ff);
        #--- Creating the walls
        for i in range(N-1):
            buildingMesh.verts.extend(groundCoverage[i][0]   - hostObject.UTMXOrigin,
                                        groundCoverage[i][1] - hostObject.UTMYOrigin ,
                                        thisBuildingZ + BHeight)
        #--- Filling the faces of the building's walls
        for i in range(N-1):
            if i < N-2:
                ff = NMesh.Face([buildingMesh.verts[i], buildingMesh.verts[i+(N-1)], buildingMesh.verts[i+N], buildingMesh.verts[i+1]]);
            else:
                ff = NMesh.Face([buildingMesh.verts[i], buildingMesh.verts[i+(N-1)], buildingMesh.verts[N-1], buildingMesh.verts[0]]);
            buildingMesh.faces.extend(ff);
        #--- Building the roof the roof the roof is on Fire... ah lala
        if (N == 5):
            # A two points roof
            longWallIs01 = 0
            roofLength   =   distance2D(buildingMesh.verts[0], buildingMesh.verts[1])
            if roofLength > distance2D(buildingMesh.verts[1], buildingMesh.verts[2]):
                longWallIs01 = 1
            else:
                longWallIs01 = 0
                roofLength   = distance2D(buildingMesh.verts[1], buildingMesh.verts[2])
            # Adding faces to the roof
            if longWallIs01 == 1:
                xroof = ((buildingMesh.verts[0]).co[0] + (buildingMesh.verts[N-2]).co[0]) / 2.0
                yroof = ((buildingMesh.verts[0]).co[1] + (buildingMesh.verts[N-2]).co[1]) / 2.0
                buildingMesh.verts.extend(xroof, yroof, thisBuildingZ + RoofHeight)
                xroof = ((buildingMesh.verts[1]).co[0] + (buildingMesh.verts[2]).co[0]) / 2.0
                yroof = ((buildingMesh.verts[1]).co[1] + (buildingMesh.verts[2]).co[1]) / 2.0
                buildingMesh.verts.extend(xroof, yroof, thisBuildingZ + RoofHeight)
                ff = NMesh.Face([buildingMesh.verts[N-1], buildingMesh.verts[N],  buildingMesh.verts[(2*(N-1))+1],  buildingMesh.verts[(2*(N-1))] ])
                buildingMesh.faces.extend(ff)
                ff = NMesh.Face([buildingMesh.verts[N-1], buildingMesh.verts[(2*(N-1))], buildingMesh.verts[N+2] ])
                buildingMesh.faces.extend(ff)
                ff = NMesh.Face([buildingMesh.verts[N], buildingMesh.verts[(2*(N-1))+1], buildingMesh.verts[N+1] ])
                buildingMesh.faces.extend(ff)
                ff = NMesh.Face([buildingMesh.verts[N+1], buildingMesh.verts[N+2],  buildingMesh.verts[(2*(N-1))],  buildingMesh.verts[(2*(N-1))+1] ])
                buildingMesh.faces.extend(ff)
            else:
                xroof = ((buildingMesh.verts[0]).co[0] + (buildingMesh.verts[1]).co[0]) / 2.0
                yroof = ((buildingMesh.verts[0]).co[1] + (buildingMesh.verts[1]).co[1]) / 2.0
                buildingMesh.verts.extend(xroof, yroof, thisBuildingZ + RoofHeight)
                xroof = ((buildingMesh.verts[2]).co[0] + (buildingMesh.verts[3]).co[0]) / 2.0
                yroof = ((buildingMesh.verts[2]).co[1] + (buildingMesh.verts[3]).co[1]) / 2.0
                buildingMesh.verts.extend(xroof, yroof, thisBuildingZ + RoofHeight)
                ff = NMesh.Face([buildingMesh.verts[N], buildingMesh.verts[N+1],  buildingMesh.verts[(2*(N-1))+1],  buildingMesh.verts[(2*(N-1))] ])
                buildingMesh.faces.extend(ff)
                ff = NMesh.Face([buildingMesh.verts[N-1], buildingMesh.verts[(2*(N-1))], buildingMesh.verts[N] ])
                buildingMesh.faces.extend(ff)
                ff = NMesh.Face([buildingMesh.verts[N+1], buildingMesh.verts[(2*(N-1))+1], buildingMesh.verts[N+2] ])
                buildingMesh.faces.extend(ff)
                ff = NMesh.Face([buildingMesh.verts[N+2], buildingMesh.verts[N-1],  buildingMesh.verts[(2*(N-1))],  buildingMesh.verts[(2*(N-1))+1] ])
                buildingMesh.faces.extend(ff)

        else:
            # A one points roof
            #xroof and yroof have already been precalculated
            xroof = 0
            yroof = 0
            for i in range((N-1)):
                xroof = xroof + (buildingMesh.verts[i]).co[0]
                yroof = yroof + (buildingMesh.verts[i]).co[1]
            #--- Average building 2D center
            xroof = xroof / float(N-1)
            yroof = yroof / float(N-1)
            #--- Adding roof top points at verts index= (2*(N-1))
            buildingMesh.verts.extend(xroof, yroof, thisBuildingZ + RoofHeight)
            #--- Making faces around the roof
            for i in range((N-1), (2*(N-1))):
                if i == ((2*(N-1))-1):
                    ff = NMesh.Face([buildingMesh.verts[i], buildingMesh.verts[N-1], buildingMesh.verts[(2*(N-1))] ])
                else:
                    ff = NMesh.Face([buildingMesh.verts[i], buildingMesh.verts[i+1], buildingMesh.verts[(2*(N-1))] ])
                buildingMesh.faces.extend(ff);

        #--- Adding material 
        buildingMesh.materials = [buildingMat]
        #--- Creating new Object in current scene
        scene = Blender.Scene.GetCurrent()
        buildingObject = scene.objects.new(buildingMesh)
        Blender.Window.Redraw()




#----------------------------------------------------------#
def LoadRoads(shp, hostObject):
    Nshapes = (shp.info())[0]
    for i in range(Nshapes):
        # Oshp is a shape object
        Oshp = shp.read_object(i)
        # We can read the vertices from the shape object
        #logging.info(Oshp.vertices()[0])


#----------------------------------------------------------#
def make_shapefile(filename):
    obj = shapelib.SHPObject(shapelib.SHPT_POLYGON, 1, [[(10, 10), (20, 10), (20, 20), (10, 10)]])
    logging.info(obj.extents())
    logging.info(obj.vertices())
    outfile = shapelib.create(filename, shapelib.SHPT_POLYGON)
    outfile.write_object(-1, obj)
    del outfile


#----------------------------------------------------------#
#
#       Test the DBF file module.
#

def make_dbf(file):
    # create a new dbf file and add three fields.
    dbf = dbflib.create(file)
    dbf.add_field("NAME", dbflib.FTString, 20, 0)
    dbf.add_field("INT", dbflib.FTInteger, 10, 0)
    dbf.add_field("FLOAT", dbflib.FTDouble, 10, 4)

#----------------------------------------------------------#
def add_dbf_records(file):
    # add some records to file
    dbf = dbflib.open(file, "r+b")
    # Records can be added as a dictionary...
    dbf.write_record(0, {'NAME': "Weatherwax", "INT":1, "FLOAT":3.1415926535})
    # ... or as a sequence
    dbf.write_record(1, ("Ogg", 2, -1000.1234))

#----------------------------------------------------------#
def list_dbf(file):
    # logging.info(the contents of a dbf file to stdout)
    dbf = dbflib.DBFFile(file)
    logging.info("%d records, %d fields" % (dbf.record_count(), dbf.field_count()))
    format = ""
    for i in range(dbf.field_count()):
        type, name, len, decc = dbf.field_info(i)
        if type == 0:
            format = format + " %%(%s)%ds" % (name, len)
        elif type == 1:
            format = format + " %%(%s)%dd" % (name, len)
        elif type == 2:
            format = format + " %%(%s)%dg" % (name, len)
    logging.info(format)
    for i in range(dbf.record_count()):
        logging.info(format % dbf.read_record(i))

#----------------------------------------------------------#
#    Main method to load shapefiles according to
#   the meaning of their contents which can be indicated
#   by shpNature. 
#
# filename: Absolute Path to the shapefile.
# hostObject: Mesh in which the information should be added
# shpNature: int indicating the nature of the shape description (Roads, buildings, etc.)
#
# We define the following values for shpNature
# shpNature : 0 : Unknown content
# shpNature : any not defined value is considered as 0.
#
# shpNature : 20 : Drivable surface
# shpNature : 21 : Common Road in hard concrete (highways, streets etc.)
# shpNature : 22 : Country tracks
#
# shpNature : 30 : Buildings with flat roofs
# shpNature : 31 : Buildings with flat roofs
#
# shpNature : 40 : Vegetation high grass
# shpNature : 41 : Vegetation woods with nice trees: feuillus
# shpNature : 42 : Vegetation woods with nice trees: coniferes
#
#----------------------------------------------------------#
def LoadShapefile(filename, shpNature, hostObject):
    # The shapelib object
    shp = []
    # open the shapefile
    if os.path.isfile(filename):
        (dirname, filerelname) = os.path.split(filename)
        (bodyname, fileext) = os.path.splitext(filerelname)
        logging.info('(DD) Is FILE OK')
        # In order to open a shapefile the shapelib needs to have the shx corresponding files
        if os.path.isfile(os.path.join(dirname, bodyname+'.SHX')) or os.path.isfile(os.path.join(dirname, bodyname+'.shx')): 
            logging.info('(DD) IS SHALELIB OK')
            shp = shapelib.ShapeFile(filename)
    else: 
        return 0

    if shp:
        Tshp =  (shp.info())[1]
        logging.info('(II) Reading shapefile with type ', Tshp ,' contents (', filerelname,') as ', shpNature)
        #-------- Loading Roads
        if (shpNature >= 20) and (shpNature <= 29):
            LoadRoads(shp, hostObject)
        #-------- Loading Buildings
        if (shpNature >= 30) and (shpNature <= 39):
            LoadBuildings(shp, hostObject)


