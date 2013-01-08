import roslib; roslib.load_manifest('sensor_msgs')
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    self.register_publisher(component_instance, function, PointCloud2)

def post_pointcloud2(self, component_instance):
    """ Publish the data of the Depth Camera as a ROS PointCloud2 message.

    """
    if not component_instance.capturing:
        return # press [Space] key to enable capturing

    points = component_instance.local_data['points']
    width = component_instance.image_width * component_instance.image_height
    size = len(points)

    pc2 = PointCloud2()
    pc2.header = self.get_ros_header(component_instance)
    pc2.height = 1
    pc2.width = width
    pc2.is_dense = False
    pc2.is_bigendian = False
    pc2.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1)]
    pc2.point_step = int(size / width)
    pc2.row_step = size

    if self.ros_memoryview_patched():
        # see patch in patches/ros_memoryview.diff
        # add at the end of your builder script:
        #   env.properties(ros_memoryview_patched=True)
        pc2.data = points
    else:
        # memoryview from PyMemoryView_FromMemory()
        # implements the buffer interface
        pc2.data = bytes(points)

    self.publish(pc2, component_instance)
