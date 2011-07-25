import logging; logger = logging.getLogger("morse." + __name__)
import GameLogic

import bpy
# Import the ontology server proxy
#import oro

import morse.sensors.camera
import morse.helpers.colors

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
        # In this case, it is the list of currently visible objects
        #  by each independent robot.
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

        visibles = self.local_data['visible_objects']

        for obj in GameLogic.trackedObjects.keys():
            visible = self._check_visible(obj)

            # Object is visible and not yet in the visible_objects list...
            if visible and obj not in visibles:
                self.local_data['visible_objects'].append(obj)
                # Scale the object to show it is visible
                #obj.localScale = [1.2, 1.2, 1.2]
                logger.info("Semantic: {0}, ({1}) just appeared".format(obj.name, obj['Description']))

            # Object is not visible and was in the visible_objects list...
            if not visible and obj in visibles:
                self.local_data['visible_objects'].remove(obj)
                # Return the object to normal size
                #  when it is no longer visible
                #obj.localScale = [1.0, 1.0, 1.0]
                logger.debug("Semantic: {0}, ({1}) just disappeared".format(obj.name, obj['Description']))
        logger.debug(str(self.local_data['visible_objects']))


    def _check_visible(self, obj):
        """ Check if an object lies inside of the camera frustrum. """
        # TrackedObjects was filled at initialization
        #  with the object's bounding boxes
        bb = GameLogic.trackedObjects[obj]
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
            if obj == closest_obj:
                return True

        return False
