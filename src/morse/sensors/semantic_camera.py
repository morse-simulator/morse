import logging; logger = logging.getLogger("morse." + __name__)
import bge

import bpy
# Import the ontology server proxy
#import oro

import morse.sensors.camera
import morse.helpers.colors

from morse.helpers import passive_objects

class SemanticCameraClass(morse.sensors.camera.CameraClass):
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
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        # Locate the Blender camera object associated with this sensor
        main_obj = self.blender_obj
        for obj in main_obj.children:
            if hasattr(obj, 'lens'):
                self.blender_cam = obj
                logger.info("Camera object: {0}".format(self.blender_cam))
                break
        if not self.blender_cam:
            logger.error("no camera object associated to the semantic camera. " +\
                  "The semantic camera requires a standard Blender camera in its children.")

        # TrackedObject is a dictionary containing the list of tracked objects 
        # (->meshes with a class property set up) as keys
        #  and the bounding boxes of these objects as value.
        if not hasattr(bge.logic, 'trackedObjects'):
            logger.info('Initialization of tracked objects:')
            scene = bge.logic.getCurrentScene()
            bge.logic.trackedObjects = dict.fromkeys(passive_objects.active_objects())

            # Store the bounding box of the marked objects
            for obj in bge.logic.trackedObjects.keys():

                # bound_box returns the bounding box in local space
                #  instead of world space.
                bge.logic.trackedObjects[obj] = bpy.data.objects[obj.name].bound_box

                details = passive_objects.details(obj)
                logger.info('    - {0} (type:{1})'.format(details['label'], details['type']))


        # Prepare the exportable data of this sensor
        # In this case, it is the list of currently visible objects
        # by each independent robot.
        
        # Array for lables of visible objects        
        self.visibles = []

        self.local_data['visible_objects'] = []
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

        visibles = self.visibles

        # check which objects are visible
        for obj in bge.logic.trackedObjects.keys():
            label = passive_objects.label(obj)
            visible = self._check_visible(obj)
            obj_dict = dict([('name', label), ('position', obj.worldPosition)]) 
            # Object is visible and not yet in the visible_objects list...
            if visible and label not in visibles:
                # Create dictionary to contain object name, type, description, position and orientation
                self.visibles.append(label)
                # Scale the object to show it is visible
                #obj.localScale = [1.2, 1.2, 1.2]
                logger.debug("Semantic %s: %s just appeared" % (self.blender_obj.name, label))
            
            # Object is not visible and was in the visible_objects list...
            if not visible and label in visibles:
                self.visibles.remove(label)
                # Return the object to normal size
                #  when it is no longer visible
                #obj.localScale = [1.0, 1.0, 1.0]
                logger.debug("Semantic %s: %s just disappeared" % (self.blender_obj.name, label))
        
        # Create dictionaries
        self.local_data['visible_objects'] = []
        for obj in bge.logic.trackedObjects.keys():
            label = passive_objects.label(obj)
            if label in visibles:
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
                
        logger.debug("Visible objects: "+ str(self.local_data['visible_objects']))

        # Create dictionary to contain object name, type, description, position and orientation
        #obj_dict = dict([('name', label), ('description', ''), ('type', ''), ('position', obj.worldPosition), ('orientation', obj.worldOrientation.to_quaternion())]) 
        # Set description and type if those properties exist
        #try:
        #    obj_dict['description'] = obj['Description']
        #except KeyError:
        #    pass
        #try:
        #    obj_dict['type'] = obj['Type']
        #except KeyError:
        #    pass


    def _check_visible(self, obj):
        """ Check if an object lies inside of the camera frustrum. """
        # TrackedObjects was filled at initialization
        #  with the object's bounding boxes
        bb = bge.logic.trackedObjects[obj]
        pos = obj.position

        logger.debug("\n--- NEW TEST ---")
        logger.debug("OBJECT '{0}' AT {1}".format(obj, pos))
        logger.debug("CAMERA '{0}' AT {1}".format(self.blender_cam, self.blender_cam.position))
        logger.debug("BBOX: >{0}<".format([[bb_corner[i] + pos[i] for i in range(3)] for bb_corner in bb]))
        logger.debug("BBOX: {0}".format(bb))

        # Translate the bounding box to the current object position
        #  and check if it is in the frustrum
        if self.blender_cam.boxInsideFrustum([[bb_corner[i] + pos[i] for i in range(3)] for bb_corner in bb]) != self.blender_cam.OUTSIDE:
            # Check that there are no other objects between the camera
            #  and the selected object
            # NOTE: This is a very simple test. Hiding only the 'center'
            #  of an object will make it invisible, even if the rest is still
            #  seen from the camera
            closest_obj = self.blender_obj.rayCastTo(obj)
            if closest_obj in [obj] + list(obj.children):
                return True

        return False
