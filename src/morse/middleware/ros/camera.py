import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    component_instance.output_functions.append(function)
    self.register_publisher_name_class(self.get_topic_name(component_instance) +\
                                       "/image", Image)
    self.register_publisher_name_class(self.get_topic_name(component_instance) +\
                                       "/camera_info", CameraInfo)

    logger.info('ROS publisher for %s initialized'%component_instance.name())

def post_image(self, component_instance):
    """ Publish the data of the Camera as a ROS Image message.

    """
    if not component_instance.capturing:
        return # press [Space] key to enable capturing

    image_local = component_instance.local_data['image']

    image = Image()
    image.header = self.get_ros_header(component_instance)
    image.header.frame_id += '/base_image'
    image.height = component_instance.image_height
    image.width = component_instance.image_width
    image.encoding = 'rgba8'
    image.step = image.width * 4

    if self.ros_memoryview_patched():
        # see patch in patches/ros_memoryview.diff
        # add at the end of your builder script:
        #   env.properties(ros_memoryview_patched=True)
        image.data = memoryview(image_local)
    else:
        # VideoTexture.ImageRender implements the buffer interface
        image.data = bytes(image_local)

    # sensor_msgs/CameraInfo [ http://ros.org/wiki/rviz/DisplayTypes/Camera ]
    # fill this 3 parameters to get correcty image with stereo camera
    Tx = 0
    Ty = 0
    R = [1, 0, 0, 0, 1, 0, 0, 0, 1]

    intrinsic = component_instance.local_data['intrinsic_matrix']

    camera_info = CameraInfo()
    camera_info.header = image.header
    camera_info.height = image.height
    camera_info.width = image.width
    camera_info.distortion_model = 'plumb_bob'
    camera_info.K = [intrinsic[0][0], intrinsic[0][1], intrinsic[0][2],
                     intrinsic[1][0], intrinsic[1][1], intrinsic[1][2],
                     intrinsic[2][0], intrinsic[2][1], intrinsic[2][2]]
    camera_info.R = R
    camera_info.P = [intrinsic[0][0], intrinsic[0][1], intrinsic[0][2], Tx,
                     intrinsic[1][0], intrinsic[1][1], intrinsic[1][2], Ty,
                     intrinsic[2][0], intrinsic[2][1], intrinsic[2][2], 0]

    self.publish_topic(image, self.get_topic_name(component_instance) + "/image")
    self.publish_topic(camera_info, self.get_topic_name(component_instance) + "/camera_info")
