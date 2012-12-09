import roslib; roslib.load_manifest('sensor_msgs');
from sensor_msgs.msg import Imu

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    # get the IMU orientation to post in the ROS message
    # TODO XXX either store to_quaternion() directly, or don't use set_property
    self.set_property(component_instance, 'orientation',
                      component_instance.blender_obj.worldOrientation)

    parent_name = component_instance.robot_parent.blender_obj.name
    # Extract the Middleware parameters. additional parameter should be a dict
    try:
        frame_id = mw_data[3].get('frame_id', '/' + parent_name + '/imu')
    except:
        frame_id = '/' + parent_name + '/imu'

    self.set_property(component_instance, 'frame_id', frame_id)

    self.register_publisher(component_instance, function, Imu)

def post_imu(self, component_instance):
    """ Publish the data of the IMU sensor as a custom ROS Imu message
    """
    imu = Imu()
    imu.header = self.get_ros_header(component_instance)
    imu.header.frame_id = self.get_property(component_instance, 'frame_id')

    imu.orientation = self.get_property(component_instance, 'orientation').to_quaternion()

    imu.angular_velocity.x = component_instance.local_data['angular_velocity'][0]
    imu.angular_velocity.y = component_instance.local_data['angular_velocity'][1]
    imu.angular_velocity.z = component_instance.local_data['angular_velocity'][2]

    imu.linear_acceleration.x = component_instance.local_data['linear_acceleration'][0]
    imu.linear_acceleration.y = component_instance.local_data['linear_acceleration'][1]
    imu.linear_acceleration.z = component_instance.local_data['linear_acceleration'][2]

    self.publish(imu, component_instance)
