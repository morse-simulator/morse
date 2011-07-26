import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('sensor_msgs')
import rospy
from sensor_msgs.msg import Image

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    # Compose the name of the port, based on the parent and module names
    component_name = component_instance.blender_obj.name
    parent_name = component_instance.robot_parent.blender_obj.name

    # Add the new method to the component
    component_instance.output_functions.append(function)

    # Generate one publisher and one topic for each component that is a sensor and uses post_message
    self._topics.append(rospy.Publisher(parent_name + "/" + component_name, Image))
    self._seq = 0

    logger.info('######## ROS IMAGE PUBLISHER INITIALIZED ########')

def post_image(self, component_instance):
    """ Publish the data of the Camera as a ROS-Image message.

    """
    image_local = component_instance.local_data['image']
    if image_local == None or image_local == '' or not component_instance.capturing:
        return # press [Space] key to enable capturing

    # XXX Must not modify  image_local here, check that we can do it
    # safely at the camera level (need to fix other middleware).

    image_local.flip = True # GameLogic.video.source.flip (VideoTexture.ImageRender)
    parent_name = component_instance.robot_parent.blender_obj.name

    image = Image()
    image.header.stamp = rospy.Time.now()
    image.header.seq = self._seq
    # http://www.ros.org/wiki/geometry/CoordinateFrameConventions#Multi_Robot_Support
    image.header.frame_id = ('/' + parent_name + '/base_image')
    image.height = component_instance.image_height
    image.width = component_instance.image_width
    image.encoding = 'rgba8'
    image.step = image.width * 4
    # NOTE: Blender returns the image as a binary string encoded as RGBA
    # sensor_msgs.msg.Image.image need to be len() friendly
    # TODO patch ros-py3/common_msgs/sensor_msgs/src/sensor_msgs/msg/_Image.py
    # to be C-PyBuffer "aware" ? http://docs.python.org/c-api/buffer.html
    image.data = bytes(image_local.image) #numpy.reshape(image_local.image, (-1, image.step))
    # RGBA8 -> RGB8 ? (remove alpha channel, save h*w bytes, CPUvore ?)
    # http://wiki.blender.org/index.php/Dev:Source/GameEngine/2.49/VideoTexture
    # http://www.blender.org/documentation/blender_python_api_2_57_release/bge.types.html#bge.types.KX_Camera.useViewport

    for topic in self._topics:
        # publish the message on the correct topic
        if str(topic.name) == str("/" + parent_name + "/" + component_instance.blender_obj.name):
            topic.publish(image)

    self._seq = self._seq + 1

# TODO sensor_msgs/CameraInfo [ http://www.ros.org/wiki/rviz/DisplayTypes/Camera ]

