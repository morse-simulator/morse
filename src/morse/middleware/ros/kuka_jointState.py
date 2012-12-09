import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('sensor_msgs');
from sensor_msgs.msg import JointState

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    self.register_subscriber(component_instance, function, JointState, callback_wp)

def callback_wp(data, component_instance):
    """ this function is called as soon as Twist messages are published on the specific topic """
    logger.debug("Received JointState names: %s on topic %s"%(data.name, self.get_topic_name(component_instance)))
    logger.debug("Received JointState positons: %s on topic %s"%(data.position, self.get_topic_name(component_instance)))
    logger.debug("Received JointState velocity: %s on topic %s"%(data.velocity, self.get_topic_name(component_instance)))

    #component_instance.local_data["x"] = data.linear.x
    #component_instance.local_data["y"] = data.linear.y
    component_instance.local_data["kuka_1"] = data.position[0]
    component_instance.local_data["kuka_2"] = data.position[1]
    component_instance.local_data["kuka_3"] = data.position[2]
    component_instance.local_data["kuka_4"] = data.position[3]
    component_instance.local_data["kuka_5"] = data.position[4]
    component_instance.local_data["kuka_6"] = data.position[5]
    component_instance.local_data["kuka_7"] = data.position[6]

def read_jointState(self, component_instance):
    """ dummy function for Waypoints """
