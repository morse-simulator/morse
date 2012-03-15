import logging; logger = logging.getLogger("morse." + __name__)
import GameLogic

import bpy
# Import the ontology server proxy
#import oro

import morse.sensors.camera
import morse.helpers.colors

from morse.helpers import passive_objects

class ObjectTrackerClass(morse.sensors.camera.CameraClass):
    """
    This module implements an "object tracker" sensor for MORSE

    This special camera returns the list of all passive objects in the environment, 
    with the type of object and the colour of the object.
    This camera is able to recognise objects marked with a 'Object' property
    An additional 'Description' property can be set that defines the object 
    category (like 'book', 'box', 'glass'...). If this property is not set, the 
    default value 'object' is used instead.

    Other such high-level information (the semantic description of the scene)
    can be added.

    The object tracker is based on the semantic camera by Severin Lemaignan

    Version: 1.0
    Date: 16 Nov. 2009
    Author: Michael Karg, Severin Lemaignan <kargm@in.tum.de><severin.lemaignan@laas.fr>

    Copyright LAAS-CNRS 2009
    """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        # TrackedObject is a dictionary containing the list of tracked objects 
        # (->meshes with a class property set up) as keys
        #  and the bounding boxes of these objects as value.
        if not hasattr(GameLogic, 'trackedObjects'):
            logger.info('Initialization of tracked objects:')
            scene = GameLogic.getCurrentScene()
            GameLogic.trackedObjects = dict.fromkeys(passive_objects.active_objects())

            # Store the bounding box of the marked objects
            for obj in GameLogic.trackedObjects.keys():

                # bound_box returns the bounding box in local space
                #  instead of world space.
                GameLogic.trackedObjects[obj] = bpy.data.objects[obj.name].bound_box

                details = passive_objects.details(obj)
                logger.info('    - {0} (type:{1})'.format(details['label'], details['type']))


        # Prepare the exportable data of this sensor
        self.local_data['tracked_objects'] = []
        # Variable to indicate this is a camera
        self.semantic_tag = True

        logger.info('Component initialized')


    def default_action(self):
        """ Do the actual semantic 'grab'.

        Iterate over all the tracked objects,
        and check if they are visible for the robot.
        Visible objects must have a boundin box and be active
        for physical simulation (have the 'Actor' checkbox selected)
        """
        # Call the action of the parent class
        super(self.__class__,self).default_action()

        # Create dictionaries
        self.local_data['tracked_objects'] = []
        for obj in GameLogic.trackedObjects.keys():
            label = passive_objects.label(obj)

            # Create dictionary to contain object name, type, description, position and orientation
            obj_dict = dict([('name', label), ('description', ''), ('type', ''), ('position', obj.worldPosition), ('orientation', obj.worldOrientation.to_quaternion())])  
                # Set description and type if those properties exist
            try:
                obj_dict['description'] = obj['Description']
            except KeyError:
                pass
            try:
                obj_dict['type'] = obj['Type']
            except KeyError:
                pass
            self.local_data['visible_objects'].append(obj_dict)
            
        logger.info("tracked objects: "+ str(self.local_data['tracked_objects']))

