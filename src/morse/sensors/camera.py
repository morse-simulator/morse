import logging; logger = logging.getLogger("morse." + __name__)
from morse.core import blenderapi
import morse.core.sensor
from morse.helpers.components import add_property


class CameraClass(morse.core.sensor.MorseSensorClass):
    """
    This class implements the configuration of the Blender's bge.texture
    module required by the different cameras, such as video or semantic.
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
    add_property('vertical_flip', False, 'Vertical_Flip')


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
        if not blenderapi.hascameras():
            logger.warning("Blender's bge.logic does not have the 'cameras' variable, \
                    something must have failed when configuring the cameras")
            return

        # Call the bge.texture method to refresh the image
        blenderapi.cameras()[self.name()].refresh(True)


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
        scene = blenderapi.scene()

        # Link the objects using bge.texture
        if not blenderapi.hascameras():
            blenderapi.initcameras()

        mat_id = blenderapi.texture().materialID(screen, material_name)
        vt_camera = blenderapi.texture().Texture(screen, mat_id)
        vt_camera.source = blenderapi.texture().ImageRender(scene, camera)

        # Set the focal length of the camera using the Game Logic Property
        camera.lens = self.image_focal
        logger.info("\tFocal length of the camera is: %s" % camera.lens)
        
        # Set the clipping distances of the camera using the Game Logic Property
        camera.near = self.near_clipping
        logger.info("\tNear clipping distance of the camera is: %s" % camera.near)
        camera.far = self.far_clipping
        logger.info("\tFar clipping distance of the camera is: %s" % camera.far)

        # Set the background to be used for the render
        vt_camera.source.background = self.bg_color
        # Define an image size. It must be powers of two. Default 512 * 512
        vt_camera.source.capsize = [self.image_width, self.image_height]
        logger.info("Camera '%s': Exporting an image of capsize: %s pixels" % \
                (self.name(), vt_camera.source.capsize))

        # Reverse the image (boolean game-property)
        # cf. bge.logic.video.source.flip (bge.texture.ImageRender)
        # http://wiki.blender.org/index.php/Dev:Source/GameEngine/2.49/VideoTexture#Setup_the_source
        vt_camera.source.flip = self.vertical_flip

        try:
            # Use the z buffer as an image texture for the camera
            if 'Zbuffer' in self.blender_obj:
                vt_camera.source.zbuff = self.blender_obj['Zbuffer']
        except AttributeError as detail:
            logger.warn("%s\nBlender does not support z buffer in images. You need to add a patch" % detail)

        try:
            # Use the z buffer as input with an array of depths
            if 'Depth' in self.blender_obj:
                vt_camera.source.depth = self.blender_obj['Depth']
        except AttributeError as detail:
            logger.warn("%s\nBlender does not support z buffer in images. You need to add a patch" % detail)

        blenderapi.cameras()[self.name()] = vt_camera
#
#        import inspect
#        logger.error("CAMERA SOURCE '%s':" % self.name())
#        for name, thing in inspect.getmembers(bge.logic.cameras[self.name()].source):
#            logger.error("\t%s = %s" % (name, thing))
