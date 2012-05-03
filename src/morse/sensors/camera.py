import logging; logger = logging.getLogger("morse." + __name__)
import bge
import morse.core.sensor


class CameraClass(morse.core.sensor.MorseSensorClass):
    """ Base class for cameras in MORSE

    This class implements the configuration of the bge.texture module
    required by the different cameras, such as video or semantic.
    """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        super(CameraClass, self).__init__(obj, parent)

        # Set the background color of the scene
        self.bg_color = [143, 143, 143, 255]
    
        # Set the values of image size from the variables
        #  in the Blender Logic Properties
        try:
            self.image_width = obj['cam_width']
            self.image_height = obj['cam_height']
            self.image_focal = obj['cam_focal']
        except KeyError:
            # Provide default values for the image properties
            # The performance is much better with power of two sizes:
            #  4, 16, 32, ... 256, 512
            logger.warning("Missing camera parameters. Using defaults")
            self.image_width = obj['cam_width'] = 256
            self.image_height = obj['cam_height'] = 256
            self.image_focal = obj['cam_focal'] = 25

        self.image_size = 4 * self.image_width * self.image_height

        self._texture_ok = False

        logger.info('Component initialized')



    def default_action(self):
        """ Update the texture image. """
        # Configure the texture settings the first time the sensor is called
        if not self._texture_ok:
            # Prepare the camera object in Blender
            self._setup_video_texture()
            self._texture_ok = True
            return

        # Exit if the cameras could not be prepared
        if not hasattr(bge.logic, 'cameras'):
            logger.warning("bge.logic does not have the 'cameras' variable, \
                    something must have failed when configuring the cameras")
            return

        # Call the bge.texture method to refresh the image
        bge.logic.cameras[self.name()].refresh(True)


    def _setup_video_texture(self):
        """ Prepare this camera to use the bge.texture module.
        Extract the references to the Blender camera and material where
        the images will be rendered.
        """
        for child in self.blender_obj.children:
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
        except UnboundLocalError as detail:
            logger.error("""
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    ERROR: The video camera could not be properly initialized.
    The children object could not be found.
    Best solution is to re-link the camera.
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    """)
            return (False)

        # Get the reference to the scene
        scene = bge.logic.getCurrentScene()

        # Link the objects using bge.texture
        if not hasattr(bge.logic, 'cameras'):
            bge.logic.cameras = {}

        mat_id = bge.texture.materialID(screen, material_name)
        vt_camera = bge.texture.Texture(screen, mat_id)
        vt_camera.source = bge.texture.ImageRender(scene, camera)

        # Set the focal length of the camera using the Game Logic Property
        camera.lens = self.image_focal
        logger.info("\tFocal length of the camera is: %s" % camera.lens)

        # Set the background to be used for the render
        vt_camera.source.background = self.bg_color
        # Define an image size. It must be powers of two. Default 512 * 512
        vt_camera.source.capsize = [self.image_width, self.image_height]
        logger.info("Camera '%s': Exporting an image of capsize: %s pixels" % \
                (self.name(), vt_camera.source.capsize))

        # Reverse the image (boolean game-property)
        # cf. bge.logic.video.source.flip (bge.texture.ImageRender)
        # http://wiki.blender.org/index.php/Dev:Source/GameEngine/2.49/VideoTexture#Setup_the_source
        if 'Vertical_Flip' in self.blender_obj: # backward compatibility
            vt_camera.source.flip = self.blender_obj['Vertical_Flip']

        bge.logic.cameras[self.name()] = vt_camera
#
#        import inspect
#        logger.error("CAMERA SOURCE '%s':" % self.name())
#        for name, thing in inspect.getmembers(bge.logic.cameras[self.name()].source):
#            logger.error("\t%s = %s" % (name, thing))
