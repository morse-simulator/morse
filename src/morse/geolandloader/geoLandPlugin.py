#######################################################################
#     This script builds a blender scene from IGN data.               #
#    You should have:                                                  #
#   - a DTM file in ASCII format (it's a must have)                   #
#   - GeoReferenced textures; tif files with their corresponding geo  #
#     references in:                                                  #
#     - GeoConcept format (.txt, .gxt): NO                            #
#     - GeoSoft Grid eXchange Format (.gxf): GOOD (but not used)      #
#     - MapInfo format (.tab, .mif): GOOD                             #
#   - Other shapefiles for buildings, roads and vegetation            #
#                                   ,                                 #
# If problems you have, get the Red Task Force                        #
#######################################################################

import Blender  # This will import the library of blender functions we will use
from Blender.BGL import *
from Blender import Draw
import os

 
###########################################
#####################  Loading procedures #
###########################################
pathprefix=os.getenv('MORSE_ROOT')
file1 = pathprefix+'/scripts/geolandloader/geoDTMLoader.py';
file2 = pathprefix+'/scripts/geolandloader/geoShapefileLoader.py';
logging.info( 'Loading file: '+file1)
execfile(os.path.normpath(file1));
logging.info( 'Loading file: '+file2)
execfile(os.path.normpath(file2));

###########################################
#####################  Other parameters   #
###########################################
dtmObject = [];
statusDTM        = '.';
statusRoads      = '.';
statusBuildings  = '.';
statusVegetation = '.';
fileDTM        = '----';
fileRoads      = '----';
fileBuildings  = '----';
fileVegetation = '----';
fileDTMShort        = '----';
fileRoadsShort      = '----';
fileBuildingsShort  = '----';
fileVegetationShort = '----';

###########################################
###################################  GUI  #
###########################################
GUIX=20;
GUIY=20;
GUIW=300;
GUIH=300;
BH=20;
BW=140;

#----------#
def draw():     # Define the GUI drawing function
    global GUIX
    global GUIY
    global GUIW
    global GUIH
    global BW
    global BH
    # Clearing the window and drawing background layout
    glClearColor(0.8,0.8,1.0,1);
    glClear(Blender.BGL.GL_COLOR_BUFFER_BIT) # This clears the window
    glColor3f(0.5,0.6,0.5) 
    glBegin(GL_LINE_LOOP)
    glVertex2i(GUIX, GUIY+GUIH)
    glVertex2i(GUIX+GUIW, GUIY+GUIH)
    glVertex2i(GUIX+GUIW, GUIY)
    glVertex2i(GUIX, GUIY)
    glEnd()
    #glBegin(GL_POLYGON)
    #glVertex2i(GUIX, GUIY+GUIH-GUIH/3+5)
    #glVertex2i(GUIX+GUIW, GUIY+GUIH-GUIH/3+5)
    #glVertex2i(GUIX+GUIW, GUIY+GUIH-GUIH/3)
    #glVertex2i(GUIX, GUIY+GUIH-GUIH/3)
    #glEnd()
    glBegin(GL_LINE)
    glVertex2i(GUIX, GUIY+50)
    glVertex2i(GUIX+GUIW, GUIY+50)
    glEnd()
    # Buttons Layout
    Draw.PushButton("DTM File", 1,             (GUIX+10), (GUIY+GUIH-BH-10), BW,BH, "Loads a DTM from an ascii file.")
    Draw.PushButton("Roads ShapeFile", 2,      (GUIX+10), (GUIY+GUIH-BH-60), BW,BH, "Loads Shapefiles with Road Data.")
    Draw.PushButton("Buildings ShapeFile", 3,  (GUIX+10), (GUIY+GUIH-BH-110), BW,BH, "Loads Shapefiles with Buildings Data.");
    Draw.PushButton("Vegetation ShapeFile", 4, (GUIX+10), (GUIY+GUIH-BH-160), BW,BH, "Loads Shapefiles with Vegetation Data.");
    Draw.PushButton("Load Scene", 5, (GUIX+GUIW/2-BW/2), (GUIY+BH/2), BW,BH, "Build the scene.");
     # Status Labels
    Draw.Label(statusDTM,        (GUIX+1), (GUIY+GUIH-BH-30), 15, BH);
    Draw.Label(statusRoads,      (GUIX+1), (GUIY+GUIH-BH-80), 15, BH);
    Draw.Label(statusBuildings,  (GUIX+1), (GUIY+GUIH-BH-130), 15, BH);
    Draw.Label(statusVegetation, (GUIX+1), (GUIY+GUIH-BH-180), 15, BH);
     # File Labels
    Draw.Label(fileDTMShort,        (GUIX+10), (GUIY+GUIH-BH-30), 250, BH);
    Draw.Label(fileRoadsShort,      (GUIX+10), (GUIY+GUIH-BH-80), 250, BH);
    Draw.Label(fileBuildingsShort,  (GUIX+10), (GUIY+GUIH-BH-130), 250, BH);
    Draw.Label(fileVegetationShort, (GUIX+10), (GUIY+GUIH-BH-180), 250, BH);

#----------#
def event(evt,val):  # Define mouse and keyboard press events
    if evt == Blender.Draw.ESCKEY: 
        Blender.Draw.Exit()    
        return 
 
#----------#
def button(evt):     # Define what to do if a button is pressed, for example:
    global GUIX
    global GUIY
    global GUIW
    global GUIH
    global BW
    global BH
    global pathprefix;
    global dtmObject;
    global statusDTM;
    global statusRoads;
    global statusBuildings;
    global statusVegetation;
    global fileDTM;
    global fileRoads;
    global fileBuildings;
    global fileVegetation;
    global fileDTMShort;
    global fileRoadsShort;
    global fileBuildingsShort;
    global fileVegetationShort;
    #------------------------------ DTM Button
    if evt == 1: # DTM File Chooser
        fileDTM = Draw.PupStrInput("DTM File: ", "/home/rod/caylus-IGN-data/archive/MNT/EXTR_DEPT82.asc", 100);
        (dirname, fileDTMShort) = os.path.split(fileDTM);
        logging.info( "Chosen file:"+fileDTM)
        Blender.Window.Redraw() # This will redraw the 3d window.
    #------------------------------ Road Button
    if evt == 2: # Shapefile Road file chooser
        fileRoads = Draw.PupStrInput("Roads Shapefile: ", "/home/rod/caylus-IGN-data/archive/DONNEES/X291T092/BDTOPO_X291_SHP_L93/A_RESEAU_ROUTIER/ROUTE.SHP", 100); 
        (dirname, fileRoadsShort) = os.path.split(fileRoads);
        logging.info( "Chosen file:"+fileRoads)
        Blender.Window.Redraw();
    #------------------------------ Buildings Button
    if evt == 3: # Buildings Shapefile file chooser
        #fileBuildings = Draw.PupStrInput("Buildings Shapefile: ", "/data/caylus_SHP/BATI_REMARQUABLE.SHP", 100); 
        fileBuildings = Draw.PupStrInput("Buildings Shapefile: ", "/data/caylus_SHP/BATI_INDIFFERENCIE.SHP", 100); 
        (dirname, fileBuildingsShort) = os.path.split(fileBuildings);
        logging.info( "Chosen file:"+fileBuildings)
        Blender.Window.Redraw();
    #------------------------------ Vegetation Button
    if evt == 4: # Vegetation Shapefile file chooser
        fileVegetation = Draw.PupStrInput("Vegetation Shapefile: ", "/home/rod/caylus-IGN-data/archive/DONNEES/X291T092/BDTOPO_X291_SHP_L93/F_VEGETATION/ZONE_VEGETATION.SHP", 100); 
        (dirname, fileVegetationShort) = os.path.split(fileVegetation);
        logging.info( "Chosen file:"+fileVegetation)
        Blender.Window.Redraw();
    #------------------------------ Load Scene Button
    if evt == 5:
        statusDTM        = '#'
        statusRoads      = '#'
        statusBuildings  = '#'
        statusVegetation = '#'
        if os.path.exists(fileDTM):
            #--- Loading DTM and creating a heighmap on a regular mesh
            dtmObject = DtmObject(fileDTM);
            if dtmObject:
                statusDTM = 'o'
                Blender.Window.Redraw();
                # DTM texturing it with a mosaic of georeferenced images
                dtmObject.setOrthoTexture('/home/rod/caylus-IGN-data/BDO_RVB_0M50_TIF_LA93_D82-9Dalles-ED05/BDO_RVB_0M50_TIF_LA93_D82-9Dalles-ED05/');
            else:
                # File exists but object wasn't created
                statusDTM = '-'
                Blender.Window.Redraw();

            # Set camera's location above the DTM mean level
            camObject = Blender.Object.Get("Camera") #scene.objects.get("Camera")
            if camObject: 
                camObject.setLocation(0.0, 0.0, dtmObject.meanZ+10.0)

            #--- Loading Roads
            if os.path.exists(fileRoads): 
                statusRoads      = '-'
                Blender.Window.Redraw();
                LoadShapefile(fileRoads, 21, dtmObject);

            #--- Loading Buildings
            if os.path.exists(fileBuildings): 
                statusBuildings  = '-'
                Blender.Window.Redraw();
                LoadShapefile(fileBuildings, 31, dtmObject);
                LoadShapefile('/data/caylus_SHP/BATI_INDUSTRIEL.SHP', 31, dtmObject);
                LoadShapefile('/data/caylus_SHP/BATI_REMARQUABLE.SHP', 31, dtmObject);
            else:
                logging.info( '(WW) File %s does not exist', fileBuildings)
            
            #--- Loading Vegetation
            if os.path.exists(fileVegetation): 
                statusVegetation = '-'
                Blender.Window.Redraw();
                LoadShapefile(fileVegetation, 41, dtmObject);

            #--- Loading Buildings
            Blender.Window.Redraw();

 
#----------#
# You can now run the Graphical User Interface by typing the command:
 
Draw.Register(draw,event,button)
 
########################################### End of script
