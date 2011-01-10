# ##### BEGIN GPL LICENSE BLOCK #####
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software Foundation,
#  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
#  Written by Peter Roelants at the KULeuven Department of Mechanical Engineering
#  under supervision of Prof. Herman Bruyninckx and Koen Buys
#
# ##### END GPL LICENSE BLOCK #####

# #####
#
# Adapted from 1-1-2011 to also start working in the 2.56 Blender release
#
# #####

import re, os.path, sys, bpy   
    
#########################################################################################
##    SimCreatorData                                                                   ##
#########################################################################################
class SimCreatorData(object):
    '''
    Class for representing the data in a simulation.
    Contains references to the different objects in the scene.
    Contains a reference to the simCreator library.
    '''
    
    def __init__(self, library):
        '''
        Initialize a SimCreatorData object.
        @param library:
            reference to a SimLibrary object.
        '''
        # Variables
        self.library = library # library object that is connected to the simCreator directory
        self.environment = None # environment of the current simulation
        self.robotsInScene = [] # list with SimRobots in the current scene
        self.sensorsInScene = [] # list with SimSensors in the current scene
        self.toolsInScene = [] # list with SimTools in the current scene
        self.actorObjectCount = 0 # number of actorObject added to this simulation (used for giving unique names to all objects)
        self.availableAnchorPointList = [] # list with all the available anchors of the simulation
        self.anchorHolderNameList = ['All', 'Environment', 'Robots', 'Sensors', 'Tools'] # List with all the options in the anchorHolder dropdown menu
        self.simActorObjectTypesList = ['All', 'Robots', 'Sensors', 'Tools'] # List with all the options in the object types dropdown menu
        self.simActorObjectList = [] # list with all the simActorObject of the current scene. Needs to be updated by updateSimActorObjectList()
        
    def clearScene(self):
        '''Function that clears and removes the current scene.'''
        # Unlink and remove all objects
        for object in bpy.data.objects:
            for scene in object.users_scene:
                scene.objects.unlink(object)
            object.user_clear()
            bpy.data.objects.remove(object)
        # Remove all groups
        for group in bpy.data.groups:
            bpy.data.groups.remove(group)
        # Remove scene
        bpy.ops.scene.delete()
        # reset data
        self.robotsInScene = []
        self.sensorsInScene = []
        self.toolsInScene = []
        self.actorObjectCount = 0
        self.availableAnchorPointList = []
        self.simActorObjectList = []
        
    def updateSimActorObjectList(self):
        '''Function that updates the simActorObjectList containing all the simActorObjects in the scene.'''
        self.simActorObjectList = []
        for robot in self.robotsInScene:
            self.simActorObjectList.append(robot)
        for sensor in self.sensorsInScene:
            self.simActorObjectList.append(sensor)
        for tool in self.toolsInScene:
            self.simActorObjectList.append(tool)
        
    def updateAvailableAnchorPointsList(self):
        '''Function that updates the availableAnchorPointList containing the available Anchors in the scene.'''
        self.availableAnchorPointList =[]
        self.environment.updateAvailableAnchorsPointsList()
        for anchor in self.environment.availableAnchorPointList:
            self.availableAnchorPointList.append(anchor)
        for robot in self.robotsInScene:
            robot.updateAvailableAnchorsPointsList()
            for anchor in robot.availableAnchorPointList:
                self.availableAnchorPointList.append(anchor)
        for sensor in self.sensorsInScene:
            sensor.updateAvailableAnchorsPointsList()
            for anchor in sensor.availableAnchorPointList:
                self.availableAnchorPointList.append(anchor)
        for tool in self.toolsInScene:
            tool.updateAvailableAnchorsPointsList
            for anchor in tool.availableAnchorPointList:
                self.availableAnchorPointList.append(anchor)
    
    def getObjectFromGroup(self, group):
        '''
        Function if the given group is a group that contains a simActorObject, it returns this object.
        @param group:
            a Blender group
        '''
        for robot in self.robotsInScene:
            if robot.group == group:
                return robot
        for sensor in self.sensorsInScene:
            if sensor.group == group:
                return sensor
        for tool in self.toolsInScene:
            if tool.group == group:
                return tool
                    
    def getParentObject(self, anchorPoint):
        '''
        Function that returns the object that contains the given anchorPoint.
        @param anchorPoint:
            anchorPoint which owner will be returned.
        '''
        for anchor in self.environment.anchorPointList:
            if anchor == anchorPoint:
                return self.environment
        for robot in self.robotsInScene:
            for anchor in robot.anchorPointList:
                if anchor == anchorPoint:
                    return robot
        for sensor in self.sensorsInScene:
            for anchor in sensor.anchorPointList:
                if anchor == anchorPoint:
                    return sensor                
        for tool in self.toolsInScene:
            for anchor in tool.anchorPointList:
                if anchor == anchorPoint:
                    return tool  
    
    def rename(self, group):
        '''
        Rename the given group and objects to give it a unique name. And return this name.
        @param group:
            the group that needs to be renamed
        '''
        number = str(self.actorObjectCount)
        group.name = group.name + '.' + number
        for object in group.objects:
            object.name = object.name + '.' + number
        return group.name           

#########################################################################################
##    SimCreatorLibrary                                                                ##
#########################################################################################
class SimCreatorLibrary(object):
    '''
    Class that represents a library that holds a connection with the SimCreator directory.
    '''

    def __init__(self, directory):
        '''
        Initializes this SimCreatorLibrary.
        @param directory:
            directory of the simCreator files.
        '''
        #Variables
        self.directory = directory      # directory path
        self.environmentsList = []      # list with all the available scene files
        self.robotList = []             # list with all the available robot files
        self.sensorList = []            # list with all the available sensor files
        self.toolList = []              # list with all the available tool files
        self.controllerList = []        # list with all the available controller files
        self.middlewareList = []        # list with all the available middleware files
        self.modifiersList = []         # list with all the available modifiers files
        # check if the library has a valid directory path for a SimCreator Library
        if self.hasValidDir():
            self.setEnvironmentList()
            self.setRobotList()
            self.setSensorList()
            #self.setToolList()

    def hasValidDir(self):
        '''Function to check if the directory of this simCreator library is a valid simCreator directory.'''
        # check if path exists
        if not os.path.exists(self.directory):
            return False
        # check if path is a directory
        if not os.path.isdir(self.directory):
            return False
        # check if path is a simCreatorLibrary
        dir = os.listdir(self.directory)
        # @todo: add meaningfull error message here:
        if 'environments' not in dir:
            
            return False
        if 'robots' not in dir:
            return False
        if 'sensors' not in dir:
            return False
        if 'controllers' not in dir:
            return False
        if 'modifiers' not in dir:
            return False
        if 'middleware' not in dir:
            return False
        if 'human' not in dir:
            return False
        return True
    
    def getEnvironmentsDir(self):
        '''Function that returns the directory containing the environments'''
        return os.path.join(self.directory, 'environments')
    
    def getRobotsDir(self):
        '''Function that returns the directory containing the robots.'''
        return os.path.join(self.directory, 'robots')
    
    def getSensorsDir(self):
        '''Function that returns the directory containing the sensors.'''
        return os.path.join(self.directory, 'sensors')
    
    def getControllersDir(self):
        '''Function that returns the directory containing the tools.'''
        return os.path.join(self.directory, 'controllers')
             
    def getToolsDir(self):
        '''Function that returns the directory containing the tools.'''
        return os.path.join(self.directory, 'tools')

    def getModifiersDir(self):
        '''Function that returns the directory containing the tools.'''
        return os.path.join(self.directory, 'modifiers')
        
    def getHumanDir(self):
        '''Function that returns the directory containing the tools.'''
        return os.path.join(self.directory, 'human')    
        
    def getMiddlewareDir(self):
        '''Function that returns the directory containing the tools.'''
        return os.path.join(self.directory, 'middleware') 
               
    def getEnvironmentsPath(self, environment):
        '''
        Function that returns the path of the given environment.
        @param environment:
            the environment name of which the path will be returned.
        '''
        fileName = environment + '.blend'
        return os.path.join(self.getEnvironmentsDir(), environment, fileName)
    
    def getRobotPath(self, robot):
        '''
        Function that returns the path of the given robot.
        @param robot:
            the robot name of which the path will be returned.
        '''
        fileName = robot + '.blend'
        return os.path.join(self.getRobotsDir(), robot, fileName)
    
    def getSensorsPath(self, sensor):
        '''
        Function that returns the path of the given sensor.
        @param sensor:
            the sensor name of which the path will be returned.
        '''
        fileName = sensor + '.blend'
        return os.path.join(self.getSensorsDir(), sensor, fileName)
    
    def getToolPath(self, tool):
        '''
        Function that returns the path of the given tool.
        @param tool:
            the tool name of which the path will be returned.
        '''
        fileName = tool + '.blend'
        return os.path.join(self.getToolsDir(), tool, fileName)
        
    def getModifierPath(self, modifier):
        '''
        Function that returns the path of the given tool.
        @param modifier:
            the modifier name of which the path will be returned.
        '''
        fileName = modifier + '.blend'
        return os.path.join(self.getModifiersDir(), modifier, fileName)
    
    def getHumanPath(self, human):
        '''
        Function that returns the path of the given tool.
        @param human:
            the human name of which the path will be returned.
        '''
        fileName = human + '.blend'
        return os.path.join(self.getHumanDir(), human, fileName)
    
    def getMiddlewarePath(self, middleware):
        '''
        Function that returns the path of the given tool.
        @param middleware:
            the middleware name of which the path will be returned.
        '''
        fileName = middleware + '.blend'
        return os.path.join(self.getMiddlewareDir(), middleware, fileName)
    
    def setEnvironmentList(self):
        '''Function that searches the Environments directory and appends the environments to self.environmentsList.'''
        self.environmentsList = []
        # search and extract valid files
        for dir in os.listdir(self.getEnvironmentsDir()):
            path = os.path.join(self.getEnvironmentsDir(), dir)
            if os.path.isdir(path):
                for file in os.listdir(path):
                    # check if file is .blend file
                    if re.search('\.blend$', file):
                        file = re.sub('\.blend$', '', file)
                        # append valid files
                        self.environmentsList.append(file)
                
    def setRobotList(self):
        '''Function that searches the Robots directory and appends the robots to self.robotList.'''
        self.robotList = []
        # search and extract valid files
        for dir in os.listdir(self.getRobotsDir()):
            path = os.path.join(self.getRobotsDir(), dir)
            if os.path.isdir(path):
                for file in os.listdir(path):
                    # check if file is .blend file
                    if re.search('\.blend$', file):
                        file = re.sub('\.blend$', '', file)
                        # append valid files
                        self.robotList.append(file)
                
    def setSensorList(self):
        '''Function that searches the Sensors directory and appends the sensors to self.sensorList.'''
        self.sensorList = []
        # search and extract valid files
        for dir in os.listdir(self.getSensorsDir()):
            path = os.path.join(self.getSensorsDir(), dir)
            if os.path.isdir(path):
                for file in os.listdir(path):
                    # check if file is .blend file
                    if re.search('\.blend$', file):
                        file = re.sub('\.blend$', '', file)
                        # append valid files
                        self.sensorList.append(file)
                        
    def setToolList(self):
        '''Function that searches the Tools directory and appends the tools to self.toolList.'''
        self.toolList = []
        # search and extract valid files
        for dir in os.listdir(self.getToolsDir()):
            path = os.path.join(self.getToolsDir(), dir)
            if os.path.isdir(path):
                for file in os.listdir(path):
                    # check if file is .blend file
                    if re.search('\.blend$', file):
                        file = re.sub('\.blend$', '', file)
                        # append valid files
                        self.toolList.append(file)
                        
#########################################################################################
##    SimObject                                                                        ##
#########################################################################################
class SimObject(object):
    '''
    Class representing an object in a simulation.
    '''

    def __init__(self, simData):
        '''
        Initialize this simObject.
        @param simData:
            reference to a SimCreatorData object that holds the data of the simulation this object is a part of.
        '''
        self.simData = simData # reference to the SimCreatorData object
        self.anchorPointList = [] # list with all the anchorPoints of this object
        self.availableAnchorPointList = [] # list with all the available anchorPoints of this object
        self.simObjectList = [] # list with all the SimObjects connected to this object
        
    def updateAvailableAnchorsPointsList(self):
        '''Function used to update the availableAnchorPointList.'''
        self.availableAnchorPointList = []
        # iterate over all the anchors of this object
        for anchorPoint in self.anchorPointList:
            try:
                # check if anchorPoint already is connected
                if not anchorPoint['isConnected']:
                    self.availableAnchorPointList.append(anchorPoint)
            # anchorPoint hasn't got the 'isConnected' property
            except:
                anchorPoint['isConnected'] = False
                self.availableAnchorPointList.append(anchorPoint)
        
#########################################################################################
##  SimActorObject                                                                     ##
#########################################################################################
class SimActorObject(SimObject):
    '''
    Class representing an actorObject in a simulation.
    Inherits from SimObject.
    The difference between SimActorObject and SimObject is that a SimActorObject is an actor in the simulation,
    and not just a static element of the environment.
    '''
    
    def __init__(self, simData, anchorPoint, name):
        '''
        Initializes this simActorObject.
        @param simData:
            reference to a SimCreatorData object that holds the data of the simulation this object is a part of.
        @param anchorPoint:
            anchorPoint of the simObject where this simActorObject is anchored to.
        @param name:
            name of this simActorObject. The final name of a simActorObject will be renamed to give it a unique name.
        '''
        # initialize superclass
        SimObject.__init__(self, simData)
        # Variables
        self.anchorPoint = anchorPoint # reference to the anchorpoint self is anchored to
        self.parentObject = simData.getParentObject(self.anchorPoint) # SimObject where self is anchored on
        self.group = bpy.data.groups[name] # Blender group of self
        self.anchorConnection = SimActorObject.getAnchorConnection(self.group) # anchorConnection of self where is anchored self.anchorPoint
        self.name = simData.rename(self.group) # name of self
        # set parent with anchorPoint
        self.anchorConnection.parent = self.anchorPoint
        # set location (This is location from anchorPoint)
        self.anchorConnection.location = [0, 0, 0]
        # set anchorPoint as connected
        self.anchorPoint['isConnected'] = True
        # add self to the parentObject simObjectList
        self.parentObject.simObjectList.append(self)
        # initialize anchorPointList
        for object in self.group.objects:
            # check if object is a AnchorPoint
            if re.search('^AnchorP', object.name):
                self.anchorPointList.append(object) 
    
    @staticmethod
    def getAnchorConnection(group):
        '''
        Searches the AnchorConnection in the given group and return it.
        @param group:
            The group that will be searched for a AnchorConnection
        '''
        for object in group.objects:
            # check if object is and anchorConnection
            if re.search('AnchorC', object.name):
                return object
            
    def remove(self):
        '''Removes self from the simulation.'''
        # remove all SimObjects on self
        for object in self.simObjectList:
            object.remove()
        # remove from parent
        self.parentObject.simObjectList.remove(self)
        # clear objectList
        self.simObjectList = []
        # disconnect anchor
        self.anchorPoint['isConnected'] = False
        # remove from simData
        if self in self.simData.robotsInScene:
            self.simData.robotsInScene.remove(self)
        elif self in self.simData.sensorsInScene:
            self.simData.sensorsInScene.remove(self)
        elif self in self.simData.toolsInScene:
            self.simData.toolsInScene.remove(self)
        # iterate over all the blender objects in the group
        for object in self.group.objects:
            # Unlink object
            for scene in object.users_scene:
                scene.objects.unlink(object)
            object.user_clear()
            # Remove object
            bpy.data.objects.remove(object)
        # Remove group
        bpy.data.groups.remove(self.group)
        
#########################################################################################
##  SimEnvironment                                                                     ##
#########################################################################################
class SimEnvironment(SimObject):
    '''
    Class for representing an environment of a simulation.
    Inherits from SimObject.
    '''

    def __init__(self, simData, scene):
        '''
        Initialize the environment.
        @param simData:
            reference to a SimCreatorData object that holds the data of the simulation this environment is a part of.
        @param scene:
            Scene that contains this environment
        '''
        # initialize superclass
        SimObject.__init__(self, simData)
        self.scene = scene # Contains the scene where the environment is a part of
        self.initAnchorPoints()
        
    def initAnchorPoints(self):
        '''Function used to initialize the anchorPointList of this environment.'''
        self.anchorPointList = []
        # iterate over all objects in the scene
        for object in self.scene.objects:
            if re.search('^AnchorP', object.name):
                # if anchorPoint is found, append this to the list
                self.anchorPointList.append(object)

#########################################################################################
##  SimRobot                                                                           ##
#########################################################################################
class SimRobot(SimActorObject):
    '''
    Class for representing a robot in a simulation.
    Inherits from SimActorObject.
    '''

    def __init__(self, simData, anchorPoint, name):
        '''
        Initialize this robot.
        @param simData: 
        @param simData:
            reference to a SimCreatorData object that holds the data of the simulation this robot is a part of.
        @param anchorPoint:
            anchorPoint of the simObject where this SimRobot is anchored to.
        @param name:
            name of this SimRObot. The final name of a SimRobot will be renamed to give it a unique name.
        '''
        # initialize superclass
        SimActorObject.__init__(self, simData, anchorPoint, name)
                
#########################################################################################
##  SimSensor                                                                          ##
#########################################################################################
class SimSensor(SimActorObject):
    '''
    Class to represent a sensor in a simulation.
    Inherits from SimActorObject.
    '''
    
    def __init__(self, simData, anchorPoint, name):
        '''
        Initializes this sensor.
        @param simData:
            reference to a SimCreatorData object that holds the data of the simulation this sensor is a part of.
        @param anchorPoint:
            anchorPoint of the simObject where this SimSensor is anchored to.
        @param name:
            name of this SimSensor. The final name of a SimSensor will be renamed to give it a unique name.        
        '''
        # Initialize superclass
        SimActorObject.__init__(self, simData, anchorPoint, name)
        
#########################################################################################
##    SimTool                                                                          ##
#########################################################################################
class SimTool(SimActorObject):
    '''
    Class to represent a tool in a simulation.
    Inherits from SimActorObject.
    '''

    def __init__(self, simData, anchorPoint, name):
        '''
        Initialize this tool.
        @param simData:
            reference to a SimCreatorData object that holds the data of the simulation this tool is a part of.
        @param anchorPoint:
            anchorPoint of the simObject where this SimTool is anchored to.
        @param name:
            name of this SimTool. The final name of a SimTool will be renamed to give it a unique name.     
        '''
        # Initialize the superclass
        SimActorObject.__init__(self, simData, anchorPoint, name)
        
#########################################################################################
##    SimCreatorPanel                                                                  ##
#########################################################################################
'''# Variables #######################################################################'''
print("INFO: creating simData")
simData = SimCreatorData(SimCreatorLibrary(os.path.realpath(os.path.dirname(sys.argv[0])))) # simulation Data
print("INFO: after creating simData")

'''# Robotics Panel ##################################################################'''
class ROBOTICS_PT_SimCreatorPanel(bpy.types.Panel):
    '''
    Defines a custom panel for creating a robotics simulation.
    This panel will hold all the buttons and information, it will be visible under scene in the Properties Window.
    This panel is divided in boxes, sometimes called 'subpanels.'
    '''
    bl_space_type = 'PROPERTIES' # Window type where the panel will be shown
    bl_region_type = 'WINDOW' 
    bl_context = 'scene' # Where to show panel in space_type
    bl_label = 'Simulation Creator'
    
    def draw(self, context):
        '''
        Function that draws the panel.
        Blender recognises this function to automaticly draw the panel.
        '''
        layout = self.layout # holds this panel's layout
        library = simData.library # holds a library reference
        selectDirBox = layout.box() # box for the selection of a directory
        # Check if library has a valid directory
        if not library.hasValidDir():
            selectDirBox.operator(operator='robotics.SetDirectoryButton', text='Invalid Directory, Select New')
        else:
            # Valid directory
            selectDirBox.operator(operator='robotics.SetDirectoryButton', text='Valid Directory')
            
            # Draw createBox
            createBox = layout.box()
            # Draw Select Scene Box
            selectEnvironmentBox = createBox.box()
            drawCreateEnvironmentPanel(selectEnvironmentBox, context)
            # check if there is an environment created
            if not simData.environment == None:
                # Draw add robot box
                addRobotBox = createBox.box()
                drawAddRobotBox(addRobotBox, context)
                # Draw add sensor box
                addSensorBox = createBox.box()
                drawAddSensorBox(addSensorBox, context)
                # Draw add tool box
                addToolBox = createBox.box()
                drawAddToolBox(addToolBox, context)
                
                # Draw remove box
                removeBox = layout.box()
                drawRemoveSimActorObjectBox(removeBox, context)
    
def hideview3DLines():
    '''Function that hides the relationship lines, grid and axes if 3Dview is visible on the screen.'''
    # get areas of the current screen
    areas = bpy.context.screen.areas
    for area in areas:
        if area.type == 'VIEW_3D':
            # area is the 3Dview
            area.active_space.relationship_lines = False
            area.active_space.display_floor = False
            area.active_space.display_x_axis = False
            area.active_space.display_y_axis = False
            area.active_space.display_z_axis = False
      
def cleanView():
    '''Function that cleans up the view by closing all subpanels, deselecting all items and hiding all empty's'''
    # Close all subpanels
    ROBOTICS_OT_CreateEnvironmentShowButton.creatingEnvironment = False
    ROBOTICS_OT_AddRobotSelectionButton.addingRobot = False
    ROBOTICS_OT_AddSensorSelectionButton.addingSensor = False
    ROBOTICS_OT_AddToolSelectionButton.addingTool = False
    ROBOTICS_OT_RemoveSimActorObjectSelectionButton.removingObject = False
    # Hide anchors
    for object in bpy.data.objects:
        if object.type == 'EMPTY':
            object.hide = True
    # Deselect all objects
    bpy.ops.object.select_all(action='DESELECT')
    bpy.context.scene.objects.active = None  
   
'''# SimCreator Directory Selection ##################################################'''
class ROBOTICS_OT_SetDirectoryButton(bpy.types.Operator):
    '''
    Button to go to the file browser and select a directory.
    '''
    bl_idname = 'ROBOTICS_OT_SetDirectoryButton' # robotics.SetDirectoryButton (name used to refer to this operator)
    bl_label = 'Set the library directory' # button label
    bl_description = 'Select a directory which contains the simulation library' # Tooltip
    
    # String property that gets the directory when we select it in the file browser window.
    directory = bpy.props.StringProperty(name='directory', description='getting directory', maxlen= 1024, default= '')
    
    def execute(self, context):
        '''Processes a click on ROBOTICS_OT_SetDirectoryButton in the filebrowser window.'''
        # set the directory of the library
        simData.library = SimCreatorLibrary(self.properties.directory)
        return {'FINISHED'}
        
    def invoke(self, context, event):
        '''Processes a click on ROBOTICS_OT_SetDirectoryButton, opens a filebrowser window'''
        wm = context.window_manager
        wm.fileselect_add(self)
        return {'RUNNING_MODAL'}  
  
'''# Available Anchor Points Selection ###############################################'''
def initAnchorHolderNameDropdown():
    '''
    Function that initializes the slctAnchrHldr enum.
    slctAnchrHldr is a Blender EnumProperty that can be displayed as a dropdown box,
    it holds the anchorHolderNames that can have anchorPoints.
    '''
    anchorHolderList = []
    # iterate over all holderNames and add each in a tuple to anchorHolderList
    for index, object in enumerate(simData.anchorHolderNameList):
        anchorHolderList.append((str(index), object, str(index)))
    # create an EnumProperty which can be used by the dropdown box to display the different holderNames
    bpy.types.Scene.EnumProperty(attr='slctAnchrHldr', name='holders', 
                                 description='Choose a holder', items=anchorHolderList, default='0')
    
def updateAvailableAnchorPoints():
    '''
    Function that updates the all available anchorPoints list and creates the slctAvAnchr enum.
    slctAvAnchr is a Blender EnumProperty that can be displayed as a dropdown box,
    it holds the available anchorPoints of whole the scene.
    '''
    simData.updateAvailableAnchorPointsList()
    anchorPointList = []
    # iterate over all anchorPoints and add each in a tuple to anchorPointList
    for index, object in enumerate(simData.availableAnchorPointList):
        anchorPointList.append((str(index), object.name, str(index)))
    # create an EnumProperty which can be used by the dropdown box to display the different anchorPoints
    bpy.types.Scene.EnumProperty(attr='slctAvAnchr', name='anchors', 
                                 description='Choose an anchorPoint', items=anchorPointList, default='0')
        
def updateAvailableEnvironmentAnchorPoints():
    '''
    Function that updates the available Environment AnchorPoints list and creates the slctAvEnvAnchrPnt enum.
    slctAvEnvAnchrPnt is a Blender EnumProperty that can be displayed as a dropdown box,
    it holds the available anchorPoints of the environment.
    '''
    # Update the available environment anchorPoints
    simData.environment.updateAvailableAnchorsPointsList()
    anchorPointList = []
    # iterate over all scenes and add each in a tuple to anchorPointList
    for index, object in enumerate(simData.environment.availableAnchorPointList):
        anchorPointList.append((str(index), object.name, str(index)))
    # create an EnumProperty which can be used by the dropdown box to display the different Environment anchorPoints
    bpy.types.Scene.EnumProperty(attr='slctAvEnvAnchrPnt', name='anchors', 
                                 description='Choose an anchorPoint', items=anchorPointList)

def updateAvailableRobots():
    '''
    Function that creates the slctAvRbt enum.
    slctAvRbt is a Blender Enumproperty that can be displayed as a dropdown box,
    it holds the robots in the scene.
    '''
    robotList = []
    # iterate over all robots in the scene and add each in a tuple to robotList
    for index, object in enumerate(simData.robotsInScene):
        robotList.append((str(index), object.name, str(index)))
    # create an EnumProperty which can be used by the dropdown box to display the different robots
    bpy.types.Scene.EnumProperty(attr='slctAvRbt', name='robots', 
                                 description='Choose a robot', items=robotList, default='0')

def updateAvailableRobotAnchorPoints():
    '''
    Function that creates the slctAvRbtAnchr enum. Use updateAvailableRobots() before this.
    slctAvRbtAnchr is a Blender Enumproperty that can be displayed as a dropdown box,
    it holds the available anchorPoints of the robot selected by the slctAvRbt enum.
    '''
    # get the robot selected by the slctAvRbt robot.
    robot = simData.robotsInScene[int(bpy.context.scene.slctAvRbt)]
    robot.updateAvailableAnchorsPointsList()
    anchorPointList =[]
    # iterate over all anchorPoints on the robot and add each in a tuple to anchorPointList
    for index, object in enumerate(robot.availableAnchorPointList):
        anchorPointList.append((str(index), object.name, str(index)))
    # create an EnumProperty which can be used by the dropdown box to display the different anchorPoints
    bpy.types.Scene.EnumProperty(attr='slctAvRbtAnchr', name='anchors', 
                                 description='Choose an anchorPoint', items=anchorPointList, default='0')

def updateAvailableSensors():
    '''
    Function that creates the slctAvSnsr enum.
    slctAvSnsr is a Blender EnumProperty that can be displayed as a dropdown box,
    it holds the available sensors in the scene.
    '''
    sensorList = []
    # iterate over all sensors in the scene and add each in a tuple to sensorList
    for index, object in enumerate(simData.sensorsInScene):
        sensorList.append((str(index), object.name, str(index)))
    # create an EnumProperty which can be used by the dropdown box to display the different sensors
    bpy.types.Scene.EnumProperty(attr='slctAvSnsr', name='sensors', 
                                 description='Choose a sensor', items=sensorList, default='0')
    
def updateAvailableSensorAnchorPoints():
    '''
    Function that creates the slctAvSnsrAnchr enum. Use updateAvailableSensors() before this.
    slctAvSnsrAnchr is a Blender EnumProperty that can be displayed as a dropdown box,
    it holds the available anchorPoints of the sensor selected by the slctAvSnsr enum.
    '''
    # get the sensor selected by the slctAvSnsr enum
    sensor = simData.sensorsInScene[int(bpy.context.scene.slctAvSnsr)]
    sensor.updateAvailableAnchorsPointsList()
    anchorPointList =[]
    # iterate over all anchorPoints on the sensor and add each in a tuple to anchorPointList
    for index, object in enumerate(sensor.availableAnchorPointList):
        anchorPointList.append((str(index), object.name, str(index)))
    # create an EnumProperty which can be used by the dropdown box to display the different anchorPoints
    bpy.types.Scene.EnumProperty(attr='slctAvSnsrAnchr', name='anchors', 
                                 description='Choose an anchorPoint', items=anchorPointList, default='0')

def updateAvailableTools():
    '''
    Function that creates the slctAvTl enum.
    slctAvTl is a Blender EnumProperty that can be displayed as a dropdown box,
    it holds the available tools in the scene.
    '''
    toolList = []
    # iterate over all tools in the scene and add each in a tuple to toolList
    for index, object in enumerate(simData.toolsInScene):
        toolList.append((str(index), object.name, str(index)))
    # create an EnumProperty which can be used by the dropdown box to display the different tools
    bpy.types.Scene.EnumProperty(attr='slctAvTl', name='tools', 
                                 description='Choose a tool', items=toolList, default='0')
    
def updateAvailableToolAnchorPoints():
    '''
    Function that creates the slctAvTlAnchr enum. Use updateAvailableTools() before this.
    slctAvTlAnchr is a Blender EnumProperty that can be displayed as a dropdown box,
    it holds the available anchorPoints of the tool selected by the slctAvTl enum.
    '''
    # get the tool selected by the slctAvTl enum
    tool = simData.toolsInScene[int(bpy.context.scene.slctAvTl)]
    tool.updateAvailableAnchorsPointsList()
    anchorPointList =[]
    # iterate over all anchorPoints on the tool and add each in a tuple to anchorPointList
    for index, object in enumerate(tool.availableAnchorPointList):
        anchorPointList.append((str(index), object.name, str(index)))
    # create an EnumProperty which can be used by the dropdown box to display the different anchorPoints
    bpy.types.Scene.EnumProperty(attr='slctAvTlAnchr', name='anchors', 
                                 description='Choose an anchorPoint', items=anchorPointList, default='0')

def drawAnchorPointSelectionButtons(layout, context, drawAddButtonsFunc, cancelButtonName):
    '''
    Function that draws the anchorPoint selection buttons on the given layout.
    This function can be used by different panels that need to select a specific anchorPoint.
    @param layout:
        layout to draw on.
    @param context:
        context of the draw function that calls this function.
    @param drawAddButtonsFunc:
        a function that draws add and cancel buttons at the end of a selection.
    @param cancelButtonName:
        Blender reference name of a cancelButton class to cancel the selection.
    '''
    # initialize the AnchorHolderName enum for the dropdown box
    initAnchorHolderNameDropdown()
    layout.label(text='Select an AnchorPoint:')
    # draw holder selection dropdown box
    layout.prop(data=context.scene, property='slctAnchrHldr', text='Holder')
    holderSelection = simData.anchorHolderNameList[int(bpy.context.scene.slctAnchrHldr)]
    # check if user wants to chose between all anchor points
    if holderSelection == 'All':
        # update the available anchor points
        updateAvailableAnchorPoints()
        # draw anchorPoint dropdown layout
        layout.prop(data=context.scene, property='slctAvAnchr', text='AnchorPoint')
        # check if list contains anchorPoints
        if len(simData.availableAnchorPointList) > 0:
            anchor = simData.availableAnchorPointList[int(bpy.context.scene.slctAvAnchr)]
            # select anchor in 3DView
            select3DViewAnchorPoint(anchor)
            # draw addButtons that will perform the action that is wanted
            drawAddButtonsFunc(layout, context)
        else:
            # draw CancelButton
            layout.operator(operator=cancelButtonName, text='Cancel')            
    # check if user wants to chose between environment anchor points
    elif holderSelection == 'Environment':
        if (not simData.environment == None):
            # update the available environment anchor points
            updateAvailableEnvironmentAnchorPoints()
            # draw environment anchorPoint dropdown layout
            layout.prop(data=context.scene, property='slctAvEnvAnchrPnt', text='AnchorPoint')
            # check if list contains anchorPoints
            if len(simData.environment.availableAnchorPointList) > 0:
                anchor = simData.environment.availableAnchorPointList[int(bpy.context.scene.slctAvEnvAnchrPnt)]
                # select anchor in 3DView
                select3DViewAnchorPoint(anchor)
                # draw addButtons that will perform the action that is wanted
                drawAddButtonsFunc(layout, context)
            else:
                # draw CancelButton
                layout.operator(operator=cancelButtonName, text='Cancel')
        else:
            # draw CancelButton
            layout.operator(operator=cancelButtonName, text='Cancel')
    # check if user wants to chose between robot anchor points
    elif holderSelection == 'Robots':
        # update the available robots
        updateAvailableRobots()
        # draw robot dropdown 
        layout.prop(data=context.scene, property='slctAvRbt', text='Robot')
        # check if there are robots in the scene
        if len(simData.robotsInScene) > 0:
            # update the available robot anchor points
            updateAvailableRobotAnchorPoints()
            # draw robotAnchor dropdown
            layout.prop(data=context.scene, property='slctAvRbtAnchr', text='AnchorPoint')
            robot = simData.robotsInScene[int(bpy.context.scene.slctAvRbt)]
            # check if list contains anchorPoints
            if len(robot.availableAnchorPointList) > 0:
                anchor = robot.availableAnchorPointList[int(bpy.context.scene.slctAvRbtAnchr)]
                # select anchor in 3DView
                select3DViewAnchorPoint(anchor)
                # draw addButtons that will perform the action that is wanted
                drawAddButtonsFunc(layout, context)
            else:
                # draw Cancelbutton
                layout.operator(operator=cancelButtonName, text='Cancel')
        else:
            # draw Cancelbutton
            layout.operator(operator=cancelButtonName, text='Cancel')
    elif holderSelection == 'Sensors':
        # update the available sensors
        updateAvailableSensors()
        # draw sensor dropdown
        layout.prop(data=context.scene, property='slctAvSnsr', text='Sensor')
        # check if there are sensors in the scene
        if len(simData.sensorsInScene) > 0:
            # update the available sensor anchor points
            updateAvailableSensorAnchorPoints()
            # draw sensorAnchor dropdown
            layout.prop(data=context.scene, property='slctAvSnsrAnchr', text='AnchorPoint')
            sensor = simData.sensorsInScene[int(bpy.context.scene.slctAvSnsr)]
            # check if list contains anchorPoints
            if len(sensor.availableAnchorPointList) > 0:
                anchor = sensor.availableAnchorPointList[int(bpy.context.scene.slctAvSnsrAnchr)]
                # select anchor in 3DView
                select3DViewAnchorPoint(anchor)
                # draw addButtons that will perform the action that is wanted
                drawAddButtonsFunc(layout, context, sensor.availableAnchorPointList)
            else:
                # draw CancelButton
                layout.operator(operator=cancelButtonName, text='Cancel')
        else:
            # draw CancelButton
            layout.operator(operator=cancelButtonName, text='Cancel')
    elif holderSelection == 'Tools':
        # update the available tools
        updateAvailableTools()
        # draw tool dropdown
        layout.prop(data=context.scene, property='slctAvTl', text='Tool')
        # check if there are tools in the scene
        if len(simData.toolsInScene) > 0:
            # update the available tool anchor points
            updateAvailableToolAnchorPoints()
            # draw toolAnchor dropdown
            layout.prop(data=context.scene, property='slctAvTlAnchr', text='AnchorPoint')
            tool = simData.toolsInScene[int(bpy.context.scene.slctAvTl)]
            # check if list contains anchorPoints
            if len(tool.availableAnchorPointList) > 0:
                anchor = tool.availableAnchorPointList[int(bpy.context.scene.slctAvTlAnchr)]
                # select anchor in 3DView
                select3DViewAnchorPoint(anchor)
                # draw addButtons that will perform the action that is wanted
                drawAddButtonsFunc(layout, context, tool.availableAnchorPointList)
            else:
                # draw cancelbutton
                layout.operator(operator=cancelButtonName, text='Cancel')
        else:
            # draw cancelbutton
            layout.operator(operator=cancelButtonName, text='Cancel')
            
def select3DViewAnchorPoint(anchor):
    '''
    Function that in the 3D view selects and makes active the selected robot Anchor.
    @param anchor:
        The anchor to select in the 3Dview
    '''
    # Check if the selected anchor is not the active object
    if not bpy.context.active_object == anchor:
        # Selected anchor is not the active object
        # hide anchors
        for anchorPoint in simData.availableAnchorPointList:
            anchorPoint.hide = True
        # show selected anchor
        anchor.hide = False
        # Deselect everything
        bpy.ops.object.select_all(action='DESELECT')
        # Select anchor
        anchor.select = True
        # make anchor the active object
        bpy.context.scene.objects.active = anchor
        
def getSelectedAnchorPoint():
    '''Function that returns the anchor selected by the selectAnchorPoint dropdown boxes.'''
    holderSelection = simData.anchorHolderNameList[int(bpy.context.scene.slctAnchrHldr)]
    if holderSelection == 'All':
        return simData.availableAnchorPointList[int(bpy.context.scene.slctAvAnchr)]
    elif holderSelection == 'Environment':
        return simData.environment.availableAnchorPointList[int(bpy.context.scene.slctAvEnvAnchrPnt)]
    elif holderSelection == 'Robots':
        robot = simData.robotsInScene[int(bpy.context.scene.slctAvRbt)]
        return robot.availableAnchorPointList[int(bpy.context.scene.slctAvRbtAnchr)]
    elif holderSelection == 'Sensors':
        sensor = simData.sensorsInScene[int(bpy.context.scene.slctAvSnsrAnchr)]
        return sensor.availableAnchorPointList[int(bpy.context.scene.slctAvSnsrAnchr)]
    elif holderSelection == 'Tools':
        tool = simData.toolsInScene[int(bpy.context.scene.slctAvTlAnchr)]
        return tool.availableAnchorPointList[int(bpy.context.scene.slctAvTlAnchr)]
    else:
        return None
    
'''# SimActorObject Selection ########################################################'''
def initSelectIn3DView():
    '''
    Function that initializes the slctIn3Dvw boolProperty.
    slctIn3Dvw is a Blender BoolProperty that can be displayed as a checkbox,
    if True then user wants to select an object through the 3Dview,
    if False then user wants to select an object through the dropdown selection boxes.
    '''
    bpy.types.Scene.BoolProperty(attr='slctIn3Dvw', name='slctIn3Dvw', description='Select objects in 3D view or via dropbox menu',
                                 default=False, options={'ANIMATABLE'}, subtype='NONE')
 
def initSimActorObjectTypes():
    '''
    Function that initializes the slctSmActrObjTp enum.
    slctSmActrObjTp is a Blender EnumProperty that can be displayed as a dropdown box,
    it holds strings with different types of Object types where the user can select from.
    '''
    typeList = []
    # iterate over all types and add each in a tuple to typeList
    for index, object in enumerate(simData.simActorObjectTypesList):
        typeList.append((str(index), object, str(index)))
    # create an EnumProperty which can be used by the dropdown box to display the different types
    bpy.types.Scene.EnumProperty(attr='slctSmActrObjTp', name='types', 
                                 description='Choose a object type', items=typeList, default='0')
    
def updateAllActorObjects():
    '''
    Function that creates and updates the slctAllSmActrObj enum.
    slctAllSmActrObj is a Blender EnumProperty that can be displayed as a dropdown box,
    it holds all the SimActorObjectes in the simultion where the user can choose from.
    '''
    # update the simActorObjectList
    simData.updateSimActorObjectList()
    actorObjectList = []
    # iterate over all simActorObjects and add each in a tuple to actorObjectList
    for index, object in enumerate(simData.simActorObjectList):
        actorObjectList.append((str(index), object.name, str(index)))
    # create an EnumProperty which can be used by the dropdown box to display the different simActorObjects
    bpy.types.Scene.EnumProperty(attr='slctAllSmActrObj', name='Objects', 
                                 description='Choose an object', items=actorObjectList, default='0')
    
def updateRobotObjects():
    '''
    Function that creates and updates the slctRbtObj enum.
    slctRbtObj is a Blender EnumProperty that can be displayed as a dropdown box,
    it holds all the simRobotObjects in the simulation where the user can choose from.
    '''
    robots = []
    # iterate over all robots in the scene and add each in a tuple to robots
    for index, object in enumerate(simData.robotsInScene):
        robots.append((str(index), object.name, str(index)))
    # create an EnumProperty which can be used by the dropdown box to display the different robots
    bpy.types.Scene.EnumProperty(attr='slctRbtObj', name='Robots', 
                                 description='Choose a robot', items=robots, default='0')
    
def updateSensorObjects():
    '''
    Function that creates and updates the slctSnsrObj enum.
    slctSnsrObj is a Blender EnumProperty that can be displayed as a dropdown box,
    it holds all the simSensorObjects in the simulation where the user can choose from.
    '''
    sensors = []
    # iterate over all robots in the scene and add each in a tuple to sensors
    for index, object in enumerate(simData.sensorsInScene):
        sensors.append((str(index), object.name, str(index)))
    # create an EnumProperty which can be used by the dropdown box to display the different sensors
    bpy.types.Scene.EnumProperty(attr='slctSnsrObj', name='Sensors', 
                                 description='Choose a sensor', items=sensors, default='0')
    
def updateToolObjects():
    '''
    Function that creates and updates the slctTlObj enum.
    slctTlObj is a Blender EnumProperty that can be displayed as a dropdown box,
    it holds all the simToolObjects in the simulation where the user can choose from.
    '''
    tools = []
    # iterate over all tools in the scene and add each in a tuple to tools
    for index, object in enumerate(simData.toolsInScene):
        tools.append((str(index), object.name, str(index)))
    # create an EnumProperty which can be used by the dropdown box to display the different tools
    bpy.types.Scene.EnumProperty(attr='slctTlObj', name='Tools', 
                                 description='Choose a tool', items=tools, default='0')
    
def drawSimActorObjectSelectionButtons(layout, context, drawButtonsFunc, cancelButtonName):
    '''
    Function that draws the SimActorObject selection buttons on the given layout.
    This function can be used by different panels that need to select a specific SimActorObject.
    @param layout:
        layout to draw on.
    @param context:
        context of the draw function that calls this function.
    @param drawAddButtonsFunc:
        a function that draws add and cancel buttons at the end of a selection.
    @param cancelButtonName:
        Blender reference name of a cancelButton class to cancel the selection.
    '''
    # initialize the select in 3D view boolProperty
    initSelectIn3DView()
    # show the select in 3D view checkbox
    layout.prop(data=context.scene, property='slctIn3Dvw', text='Select in 3D View')
    # check if select in 3D view checkbox is not checked
    if bpy.context.scene.slctIn3Dvw == False:
        # initialze the enum with the different object types to choose from
        initSimActorObjectTypes()
        layout.label(text='Select an Object:')
        # draw SimActorObject type selection dropdown box
        layout.prop(data=context.scene, property='slctSmActrObjTp', text='Type')
        typeSelection = simData.simActorObjectTypesList[int(bpy.context.scene.slctSmActrObjTp)]
        # check if user wants to choose between all objects
        if typeSelection == 'All':
            # update all actor objects
            updateAllActorObjects()
            # draw object select dropdown
            layout.prop(data=context.scene, property='slctAllSmActrObj', text='Object')
            # check if list contains items
            if len(simData.simActorObjectList) > 0:
                # select object in 3D view
                selectViewSimActorObject(simData.simActorObjectList[int(bpy.context.scene.slctAllSmActrObj)])
                # draw addButtons that will perform the action that is wanted
                drawButtonsFunc(layout, context)
            else:
                # draw CancelButton
                layout.operator(operator=cancelButtonName, text='Cancel') 
        # check if user wants to choose between robots
        elif typeSelection == 'Robots':
            # update robot objects
            updateRobotObjects()
            # draw robot select dropdown
            layout.prop(data=context.scene, property='slctRbtObj', text='Robot')
            # check if list contains items
            if len(simData.robotsInScene) > 0:
                # select object in 3D view
                selectViewSimActorObject(simData.robotsInScene[int(bpy.context.scene.slctRbtObj)])
                # draw addButtons that will perform the action that is wanted
                drawButtonsFunc(layout, context)
            else:
                # draw CancelButton
                layout.operator(operator=cancelButtonName, text='Cancel') 
        # check if user wants to choose between sensors
        elif typeSelection == 'Sensors':
            # update sensor objects
            updateSensorObjects()
            # draw sensor select dropdown
            layout.prop(data=context.scene, property='slctSnsrObj', text='Sensor')
            # check if list contains items
            if len(simData.sensorsInScene) > 0:
                # select object in 3D view
                selectViewSimActorObject(simData.sensorsInScene[int(bpy.context.scene.slctSnsrObj)])
                # draw addButtons that will perform the action that is wanted
                drawButtonsFunc(layout, context)
            else:
                # draw CancelButton
                layout.operator(operator=cancelButtonName, text='Cancel') 
        # check if user wants to chose between tools
        elif typeSelection == 'Tools':
            # update tool objects
            updateToolObjects()
            # draw tool select dropdown
            layout.prop(data=context.scene, property='slctTlObj', text='Tool')
            # check if list contains items
            if len(simData.toolsInScene) > 0:
                # select object in 3D view
                selectViewSimActorObject(simData.toolsInScene[int(bpy.context.scene.slctTlObj)])
                # draw addButtons that will perform the action that is wanted
                drawButtonsFunc(layout, context)
            else:
                # draw CancelButton
                layout.operator(operator=cancelButtonName, text='Cancel')
    else:
        # user wants to select object in 3D view
        layout.label(text='Right click on an object in the 3D view to select it.')
        object = getSelect3DViewObject()
        # check if there is a SimActorObject selected
        if not object == None:
            # select object in 3D view
            selectViewSimActorObject(object)
            layout.label(text='Selected Object is: ' + str(object.name))
            # draw addButtons that will perform the action that is wanted
            drawButtonsFunc(layout, context)
        else:
            # No simActorObject selected
            layout.label(text='No object selected.')
            # draw CancelButton
            layout.operator(operator=cancelButtonName, text='Cancel')

def selectViewSimActorObject(object):
    '''
    Function that selects and makes active in the 3D view the selected object.
    @param object:
        the SimActorObject to select in the 3D view
    '''
    # Check if the selected object anchorConnection is not the active object
    if not bpy.context.active_object == object.anchorConnection:
        # Selected object is not the active object
        # Deselect everything
        bpy.ops.object.select_all(action='DESELECT')
        # Select object
        for obj in object.group.objects:
            obj.select = True
        # make object anchorConnection the active object
        bpy.context.scene.objects.active = object.anchorConnection

def getSelectedSimActorObject():
    '''Function that returns the object selected by the SimActorObject Selection dropdown boxes.'''
    # check if user didn't select the object in 3D view
    if bpy.context.scene.slctIn3Dvw == False:
        typeSelection = simData.simActorObjectTypesList[int(bpy.context.scene.slctSmActrObjTp)]
        if typeSelection == 'All':
            return simData.simActorObjectList[int(bpy.context.scene.slctAllSmActrObj)]
        elif typeSelection == 'Robots':
            return simData.robotsInScene[int(bpy.context.scene.slctRbtObj)]
        elif typeSelection == 'Sensors':
            return simData.sensorsInScene[int(bpy.context.scene.slctSnsrObj)]
        elif typeSelection == 'Tools':
            return simData.toolsInScene[int(bpy.context.scene.slctTlObj)]
    else:
        # user selected object in 3D view
        return getSelect3DViewObject()
    
def getSelect3DViewObject():
    '''Function that returns the object selected in the 3DView.'''
    # check if there is an active object
    if not bpy.context.active_object == None:
        # check the groups of the active object
        for group in bpy.context.active_object.users_group:
            # get the SimActorObject from the group
            object = simData.getObjectFromGroup(group)
            if not object == None:
                return object

'''# Create Environment ##############################################################'''
def updateEnvironments():
    '''
    Function used to create and update the environments enum.
    environments is a Blender EnumProperty that can be displayed as a dropdown box,
    it holds all the available environments that can be added to a simulation.
    '''
    # update the available environment files
    simData.library.setEnvironmentList()
    environmentList = []
    # iterate over all scenes and add each in a tuple to sceneList
    for index, object in enumerate(simData.library.environmentsList):
        environmentList.append((str(index), object, str(index)))
    # create an EnumProperty which can be used by the dropdown box to display the different scenes
    bpy.types.Scene.EnumProperty(attr='environments', name='environments', 
                                 description='Choose an environment', items=environmentList, default='0')
    
def drawCreateEnvironmentPanel(layout, context):
    '''
    Function that draws the create environment buttons on the given layout.
    @param layout:
        layout to draw on.
    @param context:
        context of the drawFunction that calls this function.
    '''
    # check if user is creating a new environment
    if ROBOTICS_OT_CreateEnvironmentShowButton.creatingEnvironment:
        # emboss CreateEnvironmentShowButton
        layout.operator(operator='robotics.CreateEnvironmentShowButton', text='Create new Environment...', emboss=False)
        updateEnvironments()
        # draw dropdown box on panel
        layout.prop(data=context.scene, property='environments', text='Environments')
        # draw cancel/add buttons
        row = layout.row(align=True)
        row.operator(operator='robotics.CreateEnvironmentCancelButton', text='Cancel')
        row.operator(operator='robotics.CreateNewEnvironmentButton')
    else:
        layout.operator(operator='robotics.CreateEnvironmentShowButton', text='Create new Environment...')    
    
class ROBOTICS_OT_CreateEnvironmentShowButton(bpy.types.Operator):
    '''
    Class to represent a button that shows a subPanel that can create a new environment.
    '''
    bl_idname = 'ROBOTICS_OT_CreateEnvironmentShowButton'               # robotics.CreateEnvironmentShowButton (name used to refer to this operator)
    bl_label = 'Select an environment'                                  # button label
    bl_description = 'Show a selection of available environments'       # tooltip
    # Variables
    creatingEnvironment = False # is user creating a new scene
    
    #def poll(self, context):
    #    '''Poll function that decides if the button is greyed out or not.'''
    #    return not ROBOTICS_OT_CreateEnvironmentShowButton.creatingEnvironment
    
    def invoke(self, context, event):
        ''' Opens the create environment panel.'''
        # clean view
        cleanView()
        # open environment creation subPanel
        ROBOTICS_OT_CreateEnvironmentShowButton.creatingEnvironment = True
        return{'FINISHED'}
        
class ROBOTICS_OT_CreateEnvironmentCancelButton(bpy.types.Operator):
    '''
    Class to represent a button that closes the create environment subPanel.
    '''
    bl_idname = 'ROBOTICS_OT_CreateEnvironmentCancelButton' # robotics.CreateEnvironmentCancelButton (name used to refer to this operator)
    bl_label = 'Cancel' # button label
    bl_description = 'Cancel the environment creation.' # tooltip
    
    def invoke(self, context, event):
        '''Closes the create environment panel.'''
        # clean view
        cleanView()
        return{'FINISHED'}
    
class ROBOTICS_OT_CreateNewEnvironmentButton(bpy.types.Operator):
    '''
    Class to represent a button that creates a selected environment and removes all previous objects.
    '''
    bl_idname = 'ROBOTICS_OT_CreateNewEnvironmentButton' # robotics.CreateNewEnvironmentButton (name used to refer to this operator)
    bl_label = 'Create environment' # button label
    bl_description = 'Create the selected environment' # tooltip
    
    def invoke(self, context, event):
        '''Create the environment selected in environments.'''
        # get the environmentName selected by the dropdown box
        environmentName = simData.library.environmentsList[int(bpy.context.scene.environments)]
        # get file directory
        directory = simData.library.getEnvironmentPath(environmentName) + '\\Scene\\'
        # Unlink and remove all objects from the previous scene
        simData.clearScene()
        # Add the new environment
        bpy.ops.wm.link_append(directory=directory, link=False, filename=environmentName)
        # get the created scene
        for scene in bpy.data.scenes:
            if re.search(environmentName, scene.name):
                # Create environment reference
                simData.environment = SimEnvironment(simData, scene)
        # hide relationshiplines and axis
        hideview3DLines()
        # clean view
        cleanView()
        return{'FINISHED'}
 
'''# Add Robot #######################################################################'''
def updateRobots():
    '''
    Function used to update and create the robots enum.
    robots is a Blender EnumProperty that can be displayed as a dropdown box,
    it holds the available robots that can be added to a simulation.
    '''
    # update the list with robot files
    simData.library.setRobotList()
    robotsList = []
    # iterate over all robots and add each in a tuple to robotList
    for index, object in enumerate(simData.library.robotList):
        robotsList.append((str(index), object, str(index)))
    # create an EnumProperty which can be used by the dropdown box to display the different robots
    bpy.types.Scene.EnumProperty(attr='robots', name='robots', description='Choose a robot', items=robotsList, default='0') 

def drawAddRobotBox(box, context):
    '''
    Function that draws the add robot subPanel on the given layout.
    @param box:
        layout to draw on.
    @param context:
        context of the drawFunction that calls this function.
    '''    
    # check if user is adding a robot
    if ROBOTICS_OT_AddRobotSelectionButton.addingRobot:
        # emboss AddRobotSelectionButton
        box.operator(operator='robotics.AddRobotSelectionButton', text='Add a new robot...', emboss=False)
        # draw the anchorPoint selection dropdown boxes and buttons
        drawAnchorPointSelectionButtons(box, context, drawAddRobotButtons, 'robotics.AddRobotCancelButton')
    else:
        box.operator(operator='robotics.AddRobotSelectionButton', text='Add a new robot...')
        
def drawAddRobotButtons(layout, context):
    '''
    Function that draws the add robot buttons on the given layout.
    @param layout:
        layout to draw on.
    @param context:
        context of the drawFunction that calls this function.
    '''
    layout.label(text='Select a robot:')
    updateRobots()
    # draw robot dropdown box on panel
    layout.prop(data=context.scene, property='robots', text='Robots')
    # draw cancel/add buttons
    row = layout.row(align=True)
    row.operator(operator='robotics.AddRobotCancelButton', text='Cancel')
    row.operator(operator='robotics.AddNewRobotButton', text='Add Robot')

class ROBOTICS_OT_AddRobotSelectionButton(bpy.types.Operator):
    '''
    Class to represent a button that shows a subPanel to add a new robot to a simulation.
    '''
    bl_idname = 'ROBOTICS_OT_AddRobotSelectionButton' # robotics.AddRobotSelectionButton (name used to refer to this operator)
    bl_label = 'Select a robot' # button label
    bl_description = 'Show a selection of available robots and anchors.' # tooltip
    # Variables
    addingRobot = False # is user adding a new robot
    
    #def poll(self, context):
    #    '''Poll function that decides if the button is greyed out or not.'''
    #    return not ROBOTICS_OT_AddRobotSelectionButton.addingRobot
    
    def invoke(self, context, event):
        '''Opens the addRobotSelection box.'''
        # clean view
        cleanView()
        # open robot adding subPanel
        ROBOTICS_OT_AddRobotSelectionButton.addingRobot = True
        return{'FINISHED'}
        
class ROBOTICS_OT_AddRobotCancelButton(bpy.types.Operator):
    '''
    Class to represent a button that closes the add robot subPanel.
    '''
    bl_idname = 'ROBOTICS_OT_AddRobotCancelButton' # robotics.AddRobotCancelButton (name used to refer to this operator)
    bl_label = 'Cancel' # button label
    bl_description = 'Cancel the robot selection.' # tooltip
    
    def invoke(self, context, event):
        '''Closes the AddRobotSelection Box.'''
        # clean view
        cleanView()
        return{'FINISHED'}
        
class ROBOTICS_OT_AddNewRobotButton(bpy.types.Operator):
    '''
    Class to represent a button that adds a new robot to the simulation.
    '''
    bl_idname = 'ROBOTICS_OT_AddNewRobotButton' # robotics.AddNewRobotButton (name used to refer to this operator)
    bl_label = 'Add a new robot' # button label
    bl_description = 'Add the selected robot to the selected anchor.' # tooltip
    
    def invoke(self, context, event):
        '''Add a new robot to the simulation.'''
        robotName = simData.library.robotList[int(bpy.context.scene.robots)]
        # Get directory of robot Group
        directory = simData.library.getRobotPath(robotName) + '\\Group\\'
        # Add the selected robot to the scene
        bpy.ops.wm.link_append(directory=directory, link=False, filename=robotName)
        # Unlink and remove the duplicate group object
        object = bpy.data.objects[robotName]
        bpy.context.scene.objects.unlink(object)
        bpy.data.objects.remove(object)
        robotAnchor = getSelectedAnchorPoint()
        # Initialize a new robot object
        robot = SimRobot(simData, robotAnchor, robotName)
        # Add robot to the list containing all the robots in the scene
        simData.robotsInScene.append(robot)
        simData.actorObjectCount += 1  
        # clean view
        cleanView()
        return{'FINISHED'}
   
'''# Add Sensor ######################################################################'''
def updateSensors():
    '''
    Function used to update and create the sensors enum.
    sensors is a Blender EnumProperty that can be displayed as a dropdown box.
    it holds the available sensors that can be added to a simulation.
    '''
    sensorList = []
    # iterate over all sensors and add each sensor in a tuple to sensorList
    for index, object in enumerate(simData.library.sensorList):
        sensorList.append((str(index), object, str(index)))
    #create an EnumProperty which can be used by the dropdown box to display the different sensors
    bpy.types.Scene.EnumProperty(attr='sensors', name='sensors', description='Choose a sensor', items=sensorList, default='0')

def drawAddSensorBox(box, context):
    '''
    Function that draws the add sensor subPanel on the given layout.
    @param box:
        layout to draw on.
    @param context:
        context of the drawFunction that calls this function.
    '''
    # check if user is adding a sensor
    if ROBOTICS_OT_AddSensorSelectionButton.addingSensor:
        # emboss AddSensorSelectionButton
        box.operator(operator='robotics.AddSensorSelectionButton', text='Add a new sensor...', emboss=False)
        # draw the anchorPoint selection dropdown boxes and buttons
        drawAnchorPointSelectionButtons(box, context, drawAddSensorButtons, 'robotics.AddSensorCancelButton')
    else:
        box.operator(operator='robotics.AddSensorSelectionButton', text='Add a new sensor...')

def drawAddSensorButtons(layout, context):
    '''
    Function that draws the add sensor buttons on the given layout.
    @param layout:
        layout to draw on.
    @param context:
        context of the drawFunction that calls this function.
    '''
    layout.label(text='Select a sensor:')
    updateSensors()
    # draw sensor dropdown box on panel
    layout.prop(data=context.scene, property='sensors', text='Sensors')
    # draw cancel/add Buttons
    row = layout.row(align=True)
    row.operator(operator='robotics.AddSensorCancelButton', text='Cancel')
    row.operator(operator='robotics.AddNewSensorButton', text='Add Sensor')
    
class ROBOTICS_OT_AddSensorSelectionButton(bpy.types.Operator):
    '''
    Class to represent a button that shows a subPanel to add a new sensor to a simulation.
    '''
    bl_idname = 'ROBOTICS_OT_AddSensorSelectionButton' # robotics.AddSensorSelectionButton (name used to refer to this operator)
    bl_label = 'Select a new sensor' # button label
    bl_description = 'Show a selection of available Sensors' # tooltip
    # Variables
    addingSensor = False # is user adding a new sensor
    
    #def poll(self, context):
    #    '''Poll function that decides if the button is greyed out or not.'''
    #    return not ROBOTICS_OT_AddSensorSelectionButton.addingSensor
    
    def invoke(self, context, event):
        '''Function that opens the sensor selection box'''
        # clean view
        cleanView()
        # Open sensor adding subPanel
        ROBOTICS_OT_AddSensorSelectionButton.addingSensor = True
        return{'FINISHED'}
        
class ROBOTICS_OT_AddSensorCancelButton(bpy.types.Operator):
    '''
    Class to represent a button that closes the add sensor subPanel.
    '''
    bl_idname = 'ROBOTICS_OT_AddSensorCancelButton' # robotics.AddSensorCancelButton (name used to refer to this operator)
    bl_label = 'Cancel' # button label
    bl_description = 'Cancels the sensor selection.' # tooltip
    
    def invoke(self, context, event):
        '''Function that closes the sensor selection box.'''
        # clean view
        cleanView()
        return{'FINISHED'}

class ROBOTICS_OT_AddNewSensorButton(bpy.types.Operator):
    '''
    Class to represent a button that adds a new sensor to the simulation.
    '''
    bl_idname = 'ROBOTICS_OT_AddNewSensorButton' # robotics.AddNewSensorButton (name used to refer to this operator)
    bl_label = 'Add the sensor' # button label
    bl_description = 'Add the selected sensor to the selected anchor of the selected robot.' # tooltip
    
    def invoke(self, context, event):
        '''Add a new sensor to the simulation.'''
        sensorName = simData.library.sensorList[int(bpy.context.scene.sensors)]
        # Get the directory of the toolGroup
        directory = simData.library.getSensorsPath(sensorName) + '\\Group\\'
        # Add the selected sensor to the scene
        bpy.ops.wm.link_append(directory=directory, link=False, filename=sensorName)
        # Unlink and delete duplicate group object
        object = bpy.data.objects[sensorName]
        bpy.context.scene.objects.unlink(object)
        bpy.data.objects.remove(object)
        sensorAnchor = getSelectedAnchorPoint()
        # initialize new SimSensor object
        sensor = SimSensor(simData, sensorAnchor, sensorName)
        simData.sensorsInScene.append(sensor)
        simData.actorObjectCount += 1
        # clean view
        cleanView()
        return{'FINISHED'}    
         
'''# Add Tool ########################################################################'''
def updateTools():
    '''
    Function used to update and create the tools enum.
    tools is a Blender EnumProperty that can be displayed as a dropdown box,
    it holds the available tools that can be added to a simulation.
    '''
    toolList = []
    # iterate over all tools and add each tool in a tuple to toolList
    for index, object in enumerate(simData.library.toolList):
        toolList.append((str(index), object, str(index)))
    #create an EnumProperty which can be used by the dropdown box to display the different tools
    bpy.types.Scene.EnumProperty(attr='tools', name='tools', description='Choose a tool', items=toolList, default='0')

def drawAddToolBox(box, context):
    '''
    Function that draws the add tool subPanel on the given layout.
    @param box:
        layout to draw on.
    @param context:
        context of the drawFunction that calls this function.
    '''    
    # check if user is adding a tool
    if ROBOTICS_OT_AddToolSelectionButton.addingTool:
        # emboss AddToolSelectionButton
        box.operator(operator='robotics.AddToolSelectionButton', text='Add a new tool...', emboss=False)
        # draw the anchorPoint selection dropdown boxes and buttons
        drawAnchorPointSelectionButtons(box, context, drawAddToolButtons, 'robotics.AddToolCancelButton')
    else:
        box.operator(operator='robotics.AddToolSelectionButton', text='Add a new tool...')

def drawAddToolButtons(layout, context):
    '''
    Function that draws the add tool buttons on the given layout.
    @param layout:
        layout to draw on.
    @param context:
        context of the drawFunction that calls this function.
    '''
    layout.label(text='Select a tool:')
    updateTools()
    # draw tool dropdown box on panel
    layout.prop(data=context.scene, property='tools', text='Tools')
    # draw cancel/add Buttons
    row = layout.row(align=True)
    row.operator(operator='robotics.AddToolCancelButton', text='Cancel')
    row.operator(operator='robotics.AddNewToolButton', text='Add Tool')
    
class ROBOTICS_OT_AddToolSelectionButton(bpy.types.Operator):
    '''
    Class to represent a button that shows a subPanel to add a new tool to a simulation.
    '''
    bl_idname = 'ROBOTICS_OT_AddToolSelectionButton' # robotics.AddToolSelectionButton (name used to refer to this operator)
    bl_label = 'Select a new tool' # button label
    bl_description = 'Show a selection of available Tools' # tooltip
    # Variables
    addingTool = False # is user adding a new tool
    
    #def poll(self, context):
    #    '''Poll function that decides if the button is greyed out or not.'''
    #    return not ROBOTICS_OT_AddToolSelectionButton.addingTool
    
    def invoke(self, context, event):
        '''Function that opens the tool selection box'''
        # clean view
        cleanView()
        # Open tool adding subPanel
        ROBOTICS_OT_AddToolSelectionButton.addingTool = True
        return{'FINISHED'}
        
class ROBOTICS_OT_AddToolCancelButton(bpy.types.Operator):
    '''
    Class to represent a button that closes the add tool subPanel.
    '''
    bl_idname = 'ROBOTICS_OT_AddToolCancelButton' # robotics.AddToolCancelButton (name used to refer to this operator)
    bl_label = 'Cancel' # button label
    bl_description = 'Cancels the tool selection.' # tooltip
    
    def invoke(self, context, event):
        '''Function that closes the tool selection box.'''
        # clean view
        cleanView()
        return{'FINISHED'}

class ROBOTICS_OT_AddNewToolButton(bpy.types.Operator):
    '''
    Class to represent a button that adds a new tool to the simulation.
    '''
    bl_idname = 'ROBOTICS_OT_AddNewToolButton' # robotics.AddNewToolButton (name used to refer to this operator)
    bl_label = 'Add the tool' # button label
    bl_description = 'Add the selected tool to the selected anchor.' # tooltip
    
    def invoke(self, context, event):
        '''Function that adds the selected tool to the simulation.'''
        toolName = simData.library.toolList[int(bpy.context.scene.tools)]
        # Get the directory of the toolGroup
        directory = simData.library.getToolPath(toolName) + '\\Group\\'
        # Add the selected sensor to the scene
        bpy.ops.wm.link_append(directory=directory, link=False, filename=toolName)
        # Unlink and delete duplicate group object
        object = bpy.data.objects[toolName]
        bpy.context.scene.objects.unlink(object)
        bpy.data.objects.remove(object)
        toolAnchor = getSelectedAnchorPoint()
        # initialize new SimSensor object
        tool = SimTool(simData, toolAnchor, toolName)
        simData.toolsInScene.append(tool)
        simData.actorObjectCount += 1
        # clean view
        cleanView()
        return{'FINISHED'}            

'''# Remove object ###################################################################'''
def drawRemoveSimActorObjectBox(box, context):
    '''
    Function that draws the remove object subPanel on the given layout.
    @param box:
        layout to draw on.
    @param context:
        context of the drawFunction that calls this function.
    '''
    # check if user is removing an object
    if ROBOTICS_OT_RemoveSimActorObjectSelectionButton.removingObject:
        # emboss RemoveSimActorObjectSelectionButton'
        box.operator(operator='robotics.RemoveSimActorObjectSelectionButton', text='Remove an object...', emboss=False)
        # draw the object selection dropdown boxes and buttons
        drawSimActorObjectSelectionButtons(box, context, drawRemoveSimActorObjectButtons, 'robotics.RemoveSimActorObjectCancelButton')
    else:
        box.operator(operator='robotics.RemoveSimActorObjectSelectionButton', text='Remove an object...')

def drawRemoveSimActorObjectButtons(layout, context):
    '''
    Function that draws the remove object buttons on the given layout.
    @param layout:
        layout to draw on.
    @param context:
        context of the drawFunction that calls this function.
    '''
    # draw cancel/add Buttons
    row = layout.row(align=True)
    row.operator(operator='robotics.RemoveSimActorObjectCancelButton', text='Cancel')
    row.operator(operator='robotics.RemoveSimActorObjectButton', text='Remove object')
    
class ROBOTICS_OT_RemoveSimActorObjectSelectionButton(bpy.types.Operator):
    '''
    Class to represent a button that shows a subPanel to remove an object from a simulation.
    '''
    bl_idname = 'ROBOTICS_OT_RemoveSimActorObjectSelectionButton' # robotics.RemoveSimActorObjectSelectionButton (name used to refer to this operator)
    bl_label = 'Remove an object' # button label
    bl_description = 'Show a selection of objects that can be removed.' # tooltip
    # Variables
    removingObject = False # is user adding an simActorObject
    
    #def poll(self, context):
    #    '''Poll function that decides if the button is greyed out or not.'''
    #   return not ROBOTICS_OT_RemoveSimActorObjectSelectionButton.removingObject
    
    def invoke(self, context, event):
        '''Function that opens the object removing box'''
        # clean view
        cleanView()
        # open the sensor selection box
        ROBOTICS_OT_RemoveSimActorObjectSelectionButton.removingObject = True
        return{'FINISHED'}
        
class ROBOTICS_OT_RemoveSimActorObjectCancelButton(bpy.types.Operator):
    '''
    Class to represent a button that closes the remove object subPanel.
    '''
    bl_idname = 'ROBOTICS_OT_RemoveSimActorObjectCancelButton' # robotics.RemoveSimActorObjectCancelButton (name used to refer to this operator)
    bl_label = 'Cancel' # button label
    bl_description = 'Cancels the removing of an object' # tooltip
    
    def invoke(self, context, event):
        '''Function that closes the remove simActorObject box.'''
        # clean view
        cleanView()
        return{'FINISHED'}

class ROBOTICS_OT_RemoveSimActorObjectButton(bpy.types.Operator):
    '''
    Class to represent a button that removes a simActorObject from the simulation.
    '''
    bl_idname = 'ROBOTICS_OT_RemoveSimActorObjectButton' # robotics.RemoveSimActorObjectButton (name used to refer to this operator)
    bl_label = 'Remove object' # button label
    bl_description = 'Remove the selected object' # tooltip
    
    def invoke(self, context, event):
        '''Function that removes the selected object from the simulation.'''
        object = getSelectedSimActorObject()
        # remove the selected simActorObject
        object.remove()
        # clean view
        cleanView()
        return{'FINISHED'}            
        
'''# REGISTER-UNREGISTER #############################################################'''
def getClasses():
    '''Function that returns all the classes that need to be registered in Blender.'''
    CLASSES = [ROBOTICS_PT_SimCreatorPanel,
               ROBOTICS_OT_SetDirectoryButton,
               ROBOTICS_OT_CreateEnvironmentShowButton,
               ROBOTICS_OT_CreateEnvironmentCancelButton,
               ROBOTICS_OT_CreateNewEnvironmentButton,
               ROBOTICS_OT_AddRobotSelectionButton,
               ROBOTICS_OT_AddRobotCancelButton,
               ROBOTICS_OT_AddNewRobotButton,
               ROBOTICS_OT_AddSensorSelectionButton,
               ROBOTICS_OT_AddSensorCancelButton,
               ROBOTICS_OT_AddNewSensorButton,
               #ROBOTICS_OT_AddToolSelectionButton,
               #ROBOTICS_OT_AddToolCancelButton,
               #ROBOTICS_OT_AddNewToolButton,
               ROBOTICS_OT_RemoveSimActorObjectSelectionButton,
               ROBOTICS_OT_RemoveSimActorObjectCancelButton,
               ROBOTICS_OT_RemoveSimActorObjectButton]
    return CLASSES
        
def register():
    '''Register the defined classes.'''
    for definedClass in getClasses():
        bpy.types.register(definedClass)

def unregister():
    '''Unregister the defined classes.'''
    for definedClass in getClasses():
        bpy.types.unregister(definedClass)

print("INFO: Before main test")
'''# Main ############################################################################'''
if __name__ == "__main__":
    print("INFO: in main")
    # Register defined classes in Blender
    # register() # OPM: COMMENTED BY KOEN BUYS
print("INFO: after main test")
