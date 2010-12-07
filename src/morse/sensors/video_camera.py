import GameLogic
import mathutils
import morse.sensors.camera

BLENDER_HORIZONTAL_APERTURE = 32.0

class VideoCameraClass(morse.sensors.camera.CameraClass):
    """ Video capture camera

    Generates a sequence of images viewed from the camera perspective.
    """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        print ("######## VIDEO CAMERA '%s' INITIALIZING ########" % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        # Prepare the exportable data of this sensor
        self.local_data['image'] = ''

        # Prepare the intrinsic matrix for this camera.
        # Note that the matrix is stored in column major
        intrinsic = mathutils.Matrix().identity()
        alpha_u = self.image_size_X  * \
                  self.image_focal / BLENDER_HORIZONTAL_APERTURE
        intrinsic[0][0] = alpha_u
        intrinsic[1][1] = alpha_u
        intrinsic[2][0] =  self.image_size_X / 2.0
        intrinsic[2][1] =  self.image_size_Y / 2.0
        self.local_data['intrinsic_matrix'] = intrinsic

        self.capturing = False

        # Variable to indicate this is a camera
        self.camera_tag = True

        self.data_keys = ['image']

        # Initialise the copy of the data
        for variable in self.data_keys:
            self.modified_data.append(self.local_data[variable])

        print ('######## VIDEO CAMERA INITIALIZED ########')



    def default_action(self):
        """ Update the texture image. """
        # Call the action of the parent class
        super(self.__class__, self).default_action()

        # Grab an image from the texture
        if self.blender_obj['capturing']:
            # NOTE: Blender returns the image as a binary string
            #  encoded as RGBA
            if GameLogic.pythonVersion > 3:
                image_data = GameLogic.cameras[self.name].source
            else:
                image_data = GameLogic.cameras[self.name].source.image


            # Fill in the exportable data
            self.local_data['image'] = image_data
            self.capturing = True
        else:
            self.capturing = False
