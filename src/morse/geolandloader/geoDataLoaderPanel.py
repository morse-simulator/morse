
bl_addon_info = {
    'name': 'Geographical Data Loader',
    'author': 'Redouane Boumghar (LAAS - Magellium)',
    'version': '2011/09/21',
    'blender': (2, 5, 6),
    'location': 'View3D > Properties',
    'description': 'Plugin to load geographical data',
    'warning': '', # used for warning icon and text in addons panel
    'wiki_url': '',
    'tracker_url': '',
    'category': 'Geography'}

import bpy
import os
import sys
import logging
from geoDTMLoader import *

LOG_FILENAME = '/tmp/morse-geodatalandloader.log'
logging.basicConfig(filename=LOG_FILENAME,level=logging.DEBUG)

''' Geodata Loader information object '''
class GeoInfo ():
    '''
       Geodata information object. 
       This holds all information about where to find files.
       @param shape_file_dir  Root Directory containing the shape file tree
       @param dtm_file_path   Path to the digital terrain map
       @param dtm_file_format Format of the the digital terrain map
    '''
    shape_file_dir = ''
    #
    dtm_file_dir   = ''
    dtm_file_name = ''
    dtm_file_path  = ''
    dtm_file_format = ''
    #def __init__ (self):


########## Creating the GeoInfo object

geoLoaderInfo = GeoInfo();

########## BUTTONS #################################################
'''# GeoLoader File Selection ##################################################'''
class ButtonGenerateDtm (bpy.types.Operator):
    '''
    Button to check the files and call the loading of the DTM files
    '''
    bl_idname = 'morse.button_generate_dtm' # name used to refer to this operator
    bl_label = 'Generate Mesh from Digital Terrain Map' # button label
    bl_description = 'Create a digital terrain map' # Tooltip
    
    def invoke(self, context, event):
        '''ButtonGenerateDtm click: DTM generation from selected file'''
        logging.info('INVOKING'+geoLoaderInfo.dtm_file_path)
        dtmObject = DtmObject(geoLoaderInfo.dtm_file_path);
        return {'RUNNING_MODAL'}  
        
########## BUTTONS #################################################
'''# GeoLoader File Selection ##################################################'''
class ButtonFileChooserDtm (bpy.types.Operator):
    '''
    Button to check the files and call the loading of the DTM files
    '''
    bl_idname = 'morse.button_file_chooser_dtm' # name used to refer to this operator
    bl_label = 'Load Digital Terrain Map' # button label
    bl_description = 'Select a digital terrain map file' # Tooltip
    
    # String property that gets the directory when we select it in the file browser window.
    directory = bpy.props.StringProperty(name='directory', description='getting directory', maxlen= 1024, default= '')
    filename  = bpy.props.StringProperty(name='filename', description='getting filename', maxlen= 1024, default= '')
    
    def execute(self, context):
        '''Processes a click on the "submit" button in the filebrowser window.'''
        geoLoaderInfo.dtm_file_dir  = self.properties.directory
        geoLoaderInfo.dtm_file_name = self.properties.filename
        geoLoaderInfo.dtm_file_path = os.path.join(self.properties.directory, self.properties.filename)
        return {'FINISHED'}
        
    def invoke(self, context, event):
        '''Processes a click on the ButtonFileChooserDtm; opens a filebrowser window'''
        logging.info('now choosing a file');
        wm = context.window_manager
        wm.fileselect_add(self)
        return {'RUNNING_MODAL'}  
        

########## PANEL ###################################################
'''# Geo Data Panel ##################################################################'''
class PanelGeoDataLoader (bpy.types.Panel):
    '''
    Defines a custom panel for creating a robotics simulation.
    This panel will hold all the buttons and information, it will be visible under scene in the Properties Window.
    This panel is divided in boxes, sometimes called 'subpanels.'
    '''
    bl_space_type = 'PROPERTIES' # Window type where the panel will be shown
    bl_region_type = 'WINDOW' 
    bl_context = 'scene' # Where to show panel in space_type
    bl_label = 'MORSE: Geo Data Loader '
    
    def draw(self, context):
        '''
        Function that draws the panel.
        Blender recognises this function to automaticly draw the panel.
        '''
        layout = self.layout # holds this panel's layout
        selectBox = layout.box() # box for the selection of a file or generate DTM 
        # Check if geoLoaderInfo has an existing dtm file
        if not os.path.exists(geoLoaderInfo.dtm_file_path):
            selectBox.operator(operator='morse.button_file_chooser_dtm', text='Choose a DTM File')
        else:
            # Existing file
            selectBox.operator(operator='morse.button_file_chooser_dtm', text='File: '+geoLoaderInfo.dtm_file_name)
            # Adding a new box with generate buttong 
            #generateBox = selectBox.box()
            selectBox.operator(operator='morse.button_generate_dtm', text='Generate DTM')
            # Draw Select Scene Box
            #selectEnvironmentBox = createBox.box()
            #drawCreateEnvironmentPanel(selectEnvironmentBox, context)

thisPluginClasses = [
    PanelGeoDataLoader,
    ButtonFileChooserDtm,
    ButtonGenerateDtm
]

def register():
    pass
    '''Registers the needed classes. (Needed to turn addon on and off)'''
    for cl in thisPluginClasses:
        bpy.utils.register_class(cl)

def unregister():
    pass
    '''Unregisters the needed classes. (Needed to turn addon on and off)'''
    for cl in thisPluginClasses:
        bpy.utils.register_class(cl)

'''# Main ############################################################################'''
if __name__ == "__main__":
    logging.info("starting main")
    register()

