import GameLogic

if GameLogic.pythonVersion < 3:
    import Blender
else:
    import bpy
# Import the ontology server proxy
#import oro

import morse.sensors.camera
import morse.helpers.colors

class SemanticCameraClass(morse.sensors.camera.CameraClass):
    """
    This module implement a "semantic camera" sensor for the OpenRobots Simulator.

    This special camera returns the list of objects as seen by the robot's cameras,
    with unique id, possibly (if set in the objects' properties) the type of object
    and the colour of the object.
    This camera is able to recognise objects marked with a 'Description' property of type string.

    Other such high-level information (the semantic description of the scene) can be
    added.

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
        print ("######## SEMANTIC CAMERA '%s' INITIALIZING ########" % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        # Locate the Blender camera object associated with this sensor
        main_obj = self.blender_obj
        for obj in main_obj.children:
            if hasattr(obj, 'lens'):
                self.blender_cam = obj
                print ("Camera object: {0}".format(self.blender_cam))
                break

        # TrackedObject is a dictionary containing the list of tracked objects 
        # (->meshes with a class property set up) as keys
        #  and the bounding boxes of these objects as value.
        if not hasattr(GameLogic, 'trackedObjects'):
            print ('\t--- Initialization of trackedObjects variable ---')
            scene = GameLogic.getCurrentScene()
            GameLogic.trackedObjects = dict.fromkeys([ obj for obj in scene.objects if obj.getPropertyNames().count('Description')!=0 ])
            
            # Store the bounding box of the marked objects
            ################## WARNING ################## 
            # NOTE: This uses the Blender library, which has been removed
            #  in Blender 2.5. Thus this will likely break with the new version.
            for obj in GameLogic.trackedObjects.keys():
                # GetBoundBox(0) returns the bounding box in local space
                #  instead of world space.
                if GameLogic.pythonVersion < 3:
                    GameLogic.trackedObjects[obj] = Blender.Object.Get(obj.name[2:]).getBoundBox(0)
                else:
                    GameLogic.trackedObjects[obj] = bpy.data.objects[obj.name].bound_box
                print ('    - {0}'.format(obj.name))


        # Prepare the exportable data of this sensor
        # In this case, it is the list of currently visible objects by each independent robot.
        self.local_data['visible_objects'] = []
        self.data_keys = ['visible_objects']

        # Initialise the copy of the data
        for variable in self.data_keys:
            self.modified_data.append(self.local_data[variable])

        # Variable to indicate this is a camera
        self.semantic_tag = True

        print ('######## SEMANTIC CAMERA INITIALIZED ########')


    def default_action(self):
        """ Do the actual semantic 'grab'.

        Iterate over all the tracked objects,
        and check if they are visible for the robot.
        """
        # Call the action of the parent class
        super(self.__class__,self).default_action()

        visibles = self.local_data['visible_objects']

        for obj in GameLogic.trackedObjects.keys():
            visible = self._check_visible(obj)

            # Object is visible and not yet in the visible_objects list...
            if visible and visibles.count(obj) == 0:
                self.local_data['visible_objects'].append(obj)
                # Scale the object to show it is visible
                obj.localScale = [1.2, 1.2, 1.2]
                #print ("Semantic: {0}, ({1}, {2}) just appeared".format(obj.name, obj['Description'], morse.helpers.colors.retrieveHue(obj)))

            # Object is not visible and was in the visible_objects list...
            if not visible and visibles.count(obj) != 0:
                self.local_data['visible_objects'].remove(obj)
                # Return the object to normal size
                #  when it is no longer visible
                obj.localScale = [1.0, 1.0, 1.0]
                #print ("Semantic: {0}, ({1}) just disappeared".format(obj.name, obj['Description']))


    def _check_visible(self, obj):
        """ Check if an object lies inside of the camera frustrum. """
        # TrackedObjects was filled at initialization
        #  with the object's bounding boxes
        bb = GameLogic.trackedObjects[obj]
        pos = obj.position

        #print ("\n--- NEW TEST ---")
        #print ("OBJECT '{0}' AT {1}".format(obj, pos))
        #print ("CAMERA '{0}' AT {1}".format(self.blender_cam, self.blender_cam.position))
        #print ("BBOX: >{0}<".format([[bb_corner[i] + pos[i] for i in range(3)] for bb_corner in bb]))
        #print ("BBOX: {0}".format(bb))

        # Translate the bounding box to the current object position
        #  and check if it is in the frustrum
        if self.blender_cam.boxInsideFrustum([[bb_corner[i] + pos[i] for i in range(3)] for bb_corner in bb]) != self.blender_cam.OUTSIDE:
            # object is inside
            return True

        return False
