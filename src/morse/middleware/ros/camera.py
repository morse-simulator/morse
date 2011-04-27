import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('sensor_msgs')
import rospy
import std_msgs
import math
import array
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import png
import struct

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

    print('######## ROS IMAGE PUBLISHER INITIALIZED ########')

def post_image(self, component_instance):
    """ Publish the data of the Camera as a ROS-Image message.

    """
    image_local = component_instance.local_data['image']
    if image_local == None or image_local == '' or not component_instance.capturing:
        return # press [Space] key to enable capturing

    parent_name = component_instance.robot_parent.blender_obj.name
    # NOTE: Blender returns the image as a binary string encoded as RGBA
    image_data = bytes(image_local.image) # VideoTexture.ImageRender
    # http://docs.python.org/c-api/buffer.html
    image_header = std_msgs.msg._Header.Header()
    image_header.stamp = rospy.Time.now()
    # http://www.ros.org/wiki/geometry/CoordinateFrameConventions#Multi_Robot_Support
    image_header.frame_id = '/' + parent_name + '/base_image'
    image_height = component_instance.image_height
    image_width = component_instance.image_width
    image_step = image_width * 4

    #print(str(type(image_data))+str(image_data[0])+str(type(image_data[0])))
    image_data2 = [i for i in image_data] # convert from bytes to list of int
    #print(str(type(image_data2))+str(image_data2[0])+str(type(image_data2[0])))

    image = Image(header = image_header, height = image_height, 
        width = image_width, encoding = 'rgba8', #is_bigendian = 0, 
        step = image_step, data = image_data2)

    for topic in self._topics:
        # publish the message on the correct topic
        if str(topic.name) == str("/" + parent_name + "/" + component_instance.blender_obj.name):
            topic.publish(image)
            # debug
            #print(str(image_height) + 'x' + str(image_width) + ':' + str(len(image_data)) + image_header.frame_id + str(image_header.stamp.to_sec()))
            f=open('/tmp/morse_camera'+str(image_header.stamp.to_sec())+'.tmp','wb')
            f.write(image_data)
            f.close()

# https://github.com/pierriko/proteus/blob/master/rgba2png.py
# TODO sensor_msgs/CameraInfo [ http://www.ros.org/wiki/rviz/DisplayTypes/Camera ]

