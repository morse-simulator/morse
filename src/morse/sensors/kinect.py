import logging; logger = logging.getLogger("morse." + __name__)
from morse.core import blenderapi
from morse.core.sensor import Sensor
from morse.helpers.components import add_data

class Kinect(Sensor):
    """ 
    This sensor emulates the kinect output, ie both a depth image and an rgba image.
    """

    _name = "Kinect"

    add_data('depth', 'none', 'memoryview', 
              "See doc:`depth camera documentation <../sensors/depth_camera>` \
              for field **image**")
    add_data('video', 'none', 'buffer', 
             "See :doc:`video camera documentation <../sensors/video_camera>` \
              for field **image**")

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        Sensor.__init__(self, obj, parent)

        self.video_camera_name = self.name() + '.rgb'
        self.depth_camera_name = self.name() + '.depth'
        # or [child.name for child in obj.children \
        #     if child.name.startswith(self.name()+'.rgb')].pop()
        self.video_camera = None
        self.depth_camera = None
        logger.info('Component initialized, runs at %.2f Hz', self.frequency)

    def get_cameras_instance(self):
        if self.video_camera and self.depth_camera:
            return
        # Get the reference to the class instance of the depth and video cameras
        component_dict = blenderapi.persistantstorage().componentDict
        if self.video_camera_name in component_dict:
            self.video_camera = component_dict[self.video_camera_name]
        if self.depth_camera_name in component_dict:
            self.depth_camera = component_dict[self.depth_camera_name]

    @property
    def capturing(self):
        """
        Returns a boolean which indicates if the sensor actually captures
        some data
        """
        return self.depth_camera and self.depth_camera.capturing and \
               self.video_camera and self.video_camera.capturing

    def default_action(self):
        """ Get Depth and Video cameras data """
        #pass # does nothing for now
        self.get_cameras_instance()
        self.local_data['depth'] = self.depth_camera.local_data['points']
        self.local_data['video'] = self.video_camera.local_data['image']
