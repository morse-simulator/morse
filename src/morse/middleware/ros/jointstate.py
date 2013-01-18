from morse.middleware.sockets.jointstate import fill_missing_pr2_joints

import roslib; roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import JointState

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    component_instance.output_functions.append(function)
    topic = mw_data[3].get("topic", self.get_topic_name(component_instance))
    self.register_publisher_name_class(topic, JointState)

def _fill_jointstate(js, data):

    js.name = []
    js.position = []

    # collect name and positions of jointstates from sensor
    js.name += data.keys()
    js.position += data.values()

    
    # for now leaving out velocity and effort
    #js.velocity = [1, 1, 1, 1, 1, 1, 1]
    #js.effort = [50, 50, 50, 50, 50, 50, 50]

def post_jointstate(self, component_instance):
    """
    Publish the data of an armature joint state as a ROS JointState
    """

    js = JointState()
    js.header = self.get_ros_header(component_instance)

    _fill_jointstate(js, component_instance.local_data)

    self.publish(js, component_instance)

def post_pr2_jointstate(self, component_instance):
    """
    Publish the data of an armature joint state as a ROS JointState
    """
    js = JointState()
    js.header = self.get_ros_header(component_instance)

    joints =  fill_missing_pr2_joints(component_instance.local_data)

    _fill_jointstate(js, joints)

    self.publish(js, component_instance)
