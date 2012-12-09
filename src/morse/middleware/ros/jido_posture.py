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

    js.name = [
       'kuka_arm_0_joint', 'kuka_arm_1_joint', 'kuka_arm_2_joint',
       'kuka_arm_3_joint', 'kuka_arm_4_joint', 'kuka_arm_5_joint',
       'kuka_arm_6_joint', 'head_pan_joint', 'head_tilt_joint'
    ]
    js.position = [
        component_instance.local_data['seg0'],
        component_instance.local_data['seg1'],
        component_instance.local_data['seg2'],
        component_instance.local_data['seg3'],
        component_instance.local_data['seg4'],
        component_instance.local_data['seg5'],
        component_instance.local_data['seg6'],
        component_instance.local_data['pan'],
        component_instance.local_data['tilt']
    ]
    #js.velocity = [1, 1, 1, 1, 1, 1, 1]
    #js.effort = [50, 50, 50, 50, 50, 50, 50]

    self.publish(js, component_instance)
