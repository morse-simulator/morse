import logging; logger = logging.getLogger("morse." + __name__)
from morse.core import blenderapi
import morse.core.sensor
from morse.helpers.components import add_property

def copy_pose(obj_from, obj_to):
    obj_to.worldPosition = obj_from.worldPosition
    obj_to.worldOrientation = obj_from.worldOrientation

class Camera(morse.core.sensor.Sensor):
    """
    A generic camera class, which is expected to be used as a base class
    for real camera. Concrete instantiation are currently:

    - :doc:`video_camera <../sensors/video_camera>`
    - :doc:`depth_camera <../sensors/depth_camera>`
    - :doc:`semantic_camera <../sensors/semantic_camera>`

    .. note::
        The cameras make use of Blender's **bge.texture** module, which
        requires a graphic card capable of GLSL shading. Also, the 3D view
        window in Blender must be set to draw **Textured** objects.

    .. note::
        The streaming of data from this sensor can be toggled off and on by
        pressing the SPACE key during the simulation. This will affect all the
        video cameras on the scene.

        Toggling off the cameras can help make the simulation run faster,
        specially when there are several cameras. However, the lack of
        data on the stream may cause problems to some middlewares.

    .. warning::
        Contrary to most of objects in Morse, the X axis of the camera
        is not "in front" of the camera. Here, Morse follows the
        "standard convention for camera", i.e.  X and Y are in the image
        plane, and Z is in the depth axis of the camera.
    """

    _name = "Generic Camera"
    _short_desc = "Base class for cameras in MORSE"

    # Set the values of image size from the variables
    #  in the Blender Logic Properties
    add_property('image_width', 256, 'cam_width')
    add_property('image_height', 256, 'cam_height')
    add_property('image_focal', 25.0, 'cam_focal')
    add_property('near_clipping', 0.1, 'cam_near')
    add_property('far_clipping', 100.0, 'cam_far')
    add_property('vertical_flip', True, 'Vertical_Flip')
    add_property('retrieve_depth', False, 'retrieve_depth')
    add_property('retrieve_zbuffer', False, 'retrieve_zbuffer')

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)

        # Set the background color of the scene
        self.bg_color = [143, 143, 143, 255]

        self._texture_ok = False
        self._camera_running = False

        self.scene_name = 'S.%dx%d' % (self.image_width, self.image_height)

        persistantstorage = morse.core.blenderapi.persistantstorage()
        parent_name = self.robot_parent.name()
        is_parent_external = False

        for robot in persistantstorage.externalRobotDict.keys():
            if robot.name == parent_name:
                is_parent_external = True
                break

        if not is_parent_external:
            logger.info("Adding scene %s" % self.scene_name)
            blenderapi.add_scene(self.scene_name, overlay=0)
        logger.info('Component initialized, runs at %.2f Hz', self.frequency)

    def default_action(self):
        """ Update the texture image. """
        # Configure the texture settings the first time the sensor is called
        if not self._texture_ok:
            self._texture_ok = True
            if blenderapi.isfastmode():
                logger.warning("Running in fastmode! No camera support!")
            else:
                # Prepare the camera object in Blender
                self._setup_video_texture()

                # Exit if the cameras could not be prepared
                if not blenderapi.hascameras():
                    logger.warning("Blender's bge.logic does not have the 'cameras' variable, \
                            something must have failed when configuring the cameras")
                else:
                    self._camera_running = True


        if self._camera_running:
            # Update all objects pose/orientation before to refresh the image
            self._update_scene()
            # Call the bge.texture method to refresh the image
            blenderapi.cameras()[self.name()].refresh(True)

    def _update_scene(self):
        for _to, _from in self._scene_syncable_objects:
            try:
                copy_pose(_from, _to)
            except Exception as e:
                logger.warning(str(e))

    def _setup_video_texture(self):
        """ Prepare this camera to use the bge.texture module.
        Extract the references to the Blender camera and material where
        the images will be rendered.
        """
        for child in self.bge_object.children:
            # The camera object that will produce the image in Blender
            if 'CameraRobot' in child.name:
                camera = child
            # The object that contains the material where the image is rendered
            if 'CameraMesh' in child.name:
                screen = child
                # Considering it consists of a single mesh
                mesh = child.meshes[0]
                # Get the material name
                for material in mesh.materials:
                    material_index = material.getMaterialIndex()
                    mesh_material_name = mesh.getMaterialName(material_index)
                    if 'MAScreenMat' in mesh_material_name:
                        material_name = mesh_material_name

        try:
            logger.debug("\tCAMERA: %s" % camera.name)
            logger.debug("\tSCREEN: %s" % screen.name)
            logger.debug("\tMATERIAL: %s" % material_name)
        except UnboundLocalError:
            logger.error("The video camera could not be properly initialized."
                         "The children object could not be found."
                         "Best solution is to re-link the camera.")
            return False

        # Get the reference to the scene
        scene_map = blenderapi.get_scene_map()
        logger.info("Scene %s from %s"% (self.scene_name, repr(scene_map.keys()) ) )
        self._scene = scene_map[self.scene_name]
        self._morse_scene = scene_map['S.MORSE_LOGIC']

        """
        Compute the relation between objects in the current scene and
        objects in the main logic scene.

        The logic is a bit complex, as in the case of group, we can have
        objects with the same name (but different ids). So, in this
        case, we follow the hierarchy on both scene to find
        correspondance (assuming no recursive group)

        known_ids is used to track objects alreay referenced and not
        include it twice (and possibly missing the fact that the same
        name can reference multiples different objects)

        I'm definitively not sure it is correct at all, it is a really
        really dark corner of Blender :). But it seems to do the job!
        """
        self._scene_syncable_objects = []
        known_ids = set()
        for obj in self._scene.objects:
            if obj.name != '__default__cam__' and id(obj) not in known_ids:
                members = obj.groupMembers
                if not members:
                    self._scene_syncable_objects.append(
                            (obj, self._morse_scene.objects[obj.name]))
                    known_ids.add(id(obj))
                else:
                    main_members = self._morse_scene.objects[obj.name].groupMembers
                    for i in range(0, len(main_members)):
                        self._scene_syncable_objects.append(
                                (members[i], main_members[i]))
                        known_ids.add(id(members[i]))
                        childs = members[i].childrenRecursive
                        main_childs = main_members[i].childrenRecursive
                        for child in childs:
                            self._scene_syncable_objects.append(
                                    (child, main_childs[child.name]))
                            known_ids.add(id(child))


        # Link the objects using bge.texture
        if not blenderapi.hascameras():
            blenderapi.initcameras()

        mat_id = blenderapi.texture().materialID(screen, material_name)
        vt_camera = blenderapi.texture().Texture(screen, mat_id)
        vt_camera.source = blenderapi.texture().ImageRender(self._scene, camera)

        # Set the focal length of the camera using the Game Logic Property
        camera.lens = self.image_focal
        logger.info("\tFocal length of the camera is: %s" % camera.lens)

        # Set the clipping distances of the camera using the Game Logic Property
        camera.near = self.near_clipping
        logger.info("\tNear clipping distance of the camera is: %s" %
                       camera.near)
        camera.far = self.far_clipping
        logger.info("\tFar clipping distance of the camera is: %s" %
                       camera.far)

        # Set the background to be used for the render
        vt_camera.source.background = self.bg_color
        # Define an image size. It must be powers of two. Default 512 * 512
        vt_camera.source.capsize = [self.image_width, self.image_height]
        logger.info("Camera '%s': Exporting an image of capsize: %s pixels" %
                (self.name(), vt_camera.source.capsize))

        # Reverse the image (boolean game-property)
        vt_camera.source.flip = self.vertical_flip

        try:
            # Use the Z-Buffer as an image texture for the camera
            if self.retrieve_zbuffer:
                vt_camera.source.zbuff = True
            # Use the Z-Buffer as input with an array of depths
            if self.retrieve_depth:
                vt_camera.source.depth = True
        except AttributeError as detail:
            logger.warn("%s\nPlease use Blender > 2.65 for Z-Buffer support" %
                        detail)

        blenderapi.cameras()[self.name()] = vt_camera
