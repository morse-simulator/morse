import logging; logger = logging.getLogger("morse." + __name__)
import GameLogic

import bpy
# Import the ontology server proxy
#import oro

# import morse.sensors.camera
import morse.helpers.colors

class ObjectTrackerClass(morse.core.sensor.MorseSensorClass):
    """
    This module implements a "semantic camera" sensor for MORSE

    This special camera returns the list of objects as seen by the robot's
    cameras, with unique id, possibly (if set in the objects' properties)
    the type of object and the colour of the object.
    This camera is able to recognise objects marked with a 'Object' property
    An additional 'Description' property can be set that defines the object 
    category (like 'book', 'box', 'glass'...). If this property is not set, the 
    default value 'object' is used instead.

    Other such high-level information (the semantic description of the scene)
    can be added.

    Version: 1.0
    Date: 16 Nov. 2009
    Author: Severin Lemaignan <severin.lemaignan@laas.fr>

    Copyright LAAS-CNRS 2009
    """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info("######## SEMANTIC CAMERA '%s' INITIALIZING ########" % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        # Locate the Blender camera object associated with this sensor
        main_obj = self.blender_obj
  
        if not hasattr(GameLogic, 'trackedObjects'):
            logger.info('Initialization of tracked objects:')
            scene = GameLogic.getCurrentScene()
            GameLogic.trackedObjects = dict.fromkeys([ obj for obj in scene.objects if obj.getPropertyNames().count('Object')!=0 ])

            # Store the bounding box of the marked objects
            for obj in GameLogic.trackedObjects.keys():
                try:
                    obj['Description']
                except:
                    obj['Description'] = 'Object'

                # GetBoundBox(0) returns the bounding box in local space
                #  instead of world space.
                GameLogic.trackedObjects[obj] = bpy.data.objects[obj.name].bound_box
                logger.info('    - {0} (desc:{1})'.format(obj.name, obj['Description']))


        # Prepare the exportable data of this sensor
        # In this case, it is the list of all objects
        self.local_data['objects'] = []

        # Variable to indicate this is a camera
        self.semantic_tag = True

        logger.info('######## OBJECT TRACKER INITIALIZED ########')

    def default_action(self):
        """ Do the actual semantic 'grab'.

        Iterate over all the tracked objects and collect them in th objects-array
        """
        # Call the action of the parent class
        super(self.__class__,self).default_action()
        objects = self.local_data['objects']

        for obj in GameLogic.trackedObjects.keys():
            # Append object if not already in list
            if obj not in objects:
                self.local_data['objects'].append(obj)
