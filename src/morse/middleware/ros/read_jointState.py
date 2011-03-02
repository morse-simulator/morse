import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('sensor_msgs')  
import rospy
import std_msgs
from sensor_msgs.msg import JointState

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    # Compose the name of the port, based on the parent and module names
    component_name = component_instance.blender_obj.name
    parent_name = component_instance.robot_parent.blender_obj.name

    # Add the new method to the component
    component_instance.input_functions.append(function)
    self._topics.append(rospy.Subscriber(parent_name + "/" + component_name, JointState, callback_wp, component_instance))

def callback_wp(data, component_instance):
        """ Callback function reads ROS Jointstate messages for a 7 DOF robot-arm from the position array.
        The other arrays (name, velocity, ...) are not used at the moment
        """
        #print("Received JointState names: %s on topic %s"%(data.name,str("/" + component_instance.robot_parent.blender_obj.name + "/" + component_instance.blender_obj.name)))
        #print("Received JointState positons: %s on topic %s"%(data.position,str("/" + component_instance.robot_parent.blender_obj.name + "/" + component_instance.blender_obj.name)))
        #print("Received JointState velocity: %s on topic %s"%(data.velocity,str("/" + component_instance.robot_parent.blender_obj.name + "/" + component_instance.blender_obj.name)))

        component_instance.local_data["seg0"] = data.position[0]
        component_instance.local_data["seg1"] = data.position[1]
        component_instance.local_data["seg2"] = data.position[2]
        component_instance.local_data["seg3"] = data.position[3]
        component_instance.local_data["seg4"] = data.position[4]
        component_instance.local_data["seg5"] = data.position[5]
        component_instance.local_data["seg6"] = data.position[6]

def read_jointState(self, component_instance):
        """ dummy function for reading Jointstates """
