import roslib; roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import JointState

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    self.register_publisher(component_instance, function, JointState)

def post_jointState(self, component_instance):
    """ Publish the data of the Odometry-sensor as a ROS-Pose message
    """
    js = JointState()
    js.header = self.get_ros_header(component_instance)
    # collect name of jointstates from sensor
    js.name = component_instance.local_data.keys()
    # collect position information for every joint
    js.position = component_instance.local_data.values()

    # for now leaving out velocity and effort
    #js.velocity = [1, 1, 1, 1, 1, 1, 1]
    #js.effort = [50, 50, 50, 50, 50, 50, 50]

    self.publish(js, component_instance)
