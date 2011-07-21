import logging; logger = logging.getLogger("morse." + __name__)
import GameLogic
import VideoTexture
import morse.core.sensor


class CameraClass(morse.core.sensor.MorseSensorClass):
    """ Base class for cameras in MORSE

    This class implements the configuration of the VideoTexture module
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

        self.name = self.blender_obj.name

        # Prepare the camera object in Blender
        self._setup_video_texture()

        logger.info('Component initialized')



    def default_action(self):
        """ Update the texture image. """

        # Exit if the cameras could not be prepared
        if not hasattr(GameLogic, 'cameras'):
            logger.warning("GameLogic does not have the 'cameras' variable, \
                    something must have failed when configuring the cameras")
            return

        # Call the VideoTexture method to refresh the image
        GameLogic.cameras[self.name].refresh(True)


    def _setup_video_texture(self):
        """ Prepare this camera to use the VideoTexture module """
        # Get the references to this cameras materials
        #  necesary if there are more than one camera added to the scene
        screen_name = 'CameraCube' #TODO: beuuh! Hardcoded values !!
        camera_name = 'CameraRobot'
        material_name = 'MAScreenMat'
        name_len = len(self.name)
        if name_len > 4 and self.name.endswith('.00', name_len-4, name_len-1):
            extension = self.name[name_len-4:]
            screen_name = screen_name + extension
            camera_name = camera_name + extension
            #material_name = material_name + extension

        # Get the reference to the camera and screen
        scene = GameLogic.getCurrentScene()
        screen = scene.objects[screen_name]
        camera = scene.objects[camera_name]

        # Link the objects using VideoTexture
        if not hasattr(GameLogic, 'cameras'):
            GameLogic.cameras = {}

        mat_id = VideoTexture.materialID(screen, material_name)
        GameLogic.cameras[self.name] = VideoTexture.Texture(screen, mat_id)
        GameLogic.cameras[self.name].source = \
                                    VideoTexture.ImageRender(scene, camera)

        # Set the focal length of the camera using the Game Logic Property
        camera.lens = self.image_focal

        # Set the background to be used for the render
        GameLogic.cameras[self.name].source.background = self.bg_color
        # Define an image size. It must be powers of two. Default 512 * 512
        GameLogic.cameras[self.name].source.capsize = \
                [self.image_width, self.image_height]
        logger.info("Camera {0}: Exporting an image of capsize: {1} pixels". \
                format(self.name, GameLogic.cameras[self.name].source.capsize))
        logger.info("\tFocal length of the camera is: %s" % camera.lens)
