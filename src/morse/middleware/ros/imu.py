import logging; logger = logging.getLogger("morse." + __name__)

import copy
import mathutils
import math

import roslib; roslib.load_manifest('rospy'); roslib.load_manifest('nav_msgs'); roslib.load_manifest('geometry_msgs')

from morse.middleware.ros.tfMessage import tfMessage

import rospy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry

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
    if mw_data[1] == "post_velocity_twist":
        self._topics.append(rospy.Publisher(parent_name + "/" + component_name, Twist))
    else:
        self._topics.append(rospy.Publisher(parent_name + "/" + component_name, Odometry))

    self.initial_position = copy.copy(component_instance.robot_parent.position_3d)

    self.pub_tf = rospy.Publisher("/tf", tfMessage)

    self._seq = 0

    logger.info('Initialized ROS IMU sensor')

def sendTransform(self, translation, rotation, time, child, parent):
        """
        :param translation: the translation of the transformtion as a tuple (x, y, z)
        :param rotation: the rotation of the transformation as a tuple (x, y, z, w)
        :param time: the time of the transformation, as a rospy.Time()
        :param child: child frame in tf, string
        :param parent: parent frame in tf, string

        Broadcast the transformation from tf frame child to parent on ROS topic ``"/tf"``.
        """

        t = TransformStamped()
        t.header.frame_id = parent
        t.header.stamp = time
        t.child_frame_id = child
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]

        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]

        tfm = tfMessage([t])
        self.pub_tf.publish(tfm)


def post_odometry(self, component_instance):
    """ Publish the data of the IMU sensor as a ROS-Odometry message
    """
    parent = component_instance.robot_parent
    parent_name = parent.blender_obj.name
    current_time = rospy.Time.now()

    odometry = Odometry()
    odometry.header.seq = self._seq
    odometry.header.stamp = current_time
    odometry.header.frame_id = "/odom"
    odometry.child_frame_id = "/base_footprint"

    # fill pose
    pos = self.initial_position.transformation3d_with(parent.position_3d)
    odometry.pose.pose.position.x = pos.x
    odometry.pose.pose.position.y = pos.y
    odometry.pose.pose.position.z = pos.z

    euler = mathutils.Euler((pos.roll, pos.pitch, pos.yaw))
    quaternion = euler.to_quaternion()
    odometry.pose.pose.orientation = quaternion

    # fill twist
    vx, vy, vz, vroll, vpitch, vyaw = component_instance.local_data["velocity"]
    odometry.twist.twist.linear.x = math.sqrt(vx**2 + vy**2)
    odometry.twist.twist.linear.y = 0 
    odometry.twist.twist.linear.z = vz
    odometry.twist.twist.angular.x = vroll
    odometry.twist.twist.angular.y = vpitch
    odometry.twist.twist.angular.z = vyaw

    for topic in self._topics: 
        # publish the message on the correct w
        if str(topic.name) == str("/" + parent_name + "/" + component_instance.blender_obj.name): 
            topic.publish(odometry)

    # publish the odom init
    sendTransform(self, (pos.x, pos.y, pos.z),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                    rospy.Time.now(),
                    "/base_footprint",
                    "/odom")

    self._seq += 1

def post_velocity_twist(self, component_instance):
    """ Publish the data of the IMU sensor as a ROS-Twist message
    """
    parent_name = component_instance.robot_parent.blender_obj.name
    twist = Twist()
    
    # Fill twist-msg with the values from the sensor
    twist.linear.x = component_instance.local_data['velocity'][0]
    twist.linear.y = component_instance.local_data['velocity'][1]
    twist.linear.z = component_instance.local_data['velocity'][2]
    twist.angular.x =component_instance.local_data['velocity'][3]
    twist.angular.y =  component_instance.local_data['velocity'][4]
    twist.angular.z = component_instance.local_data['velocity'][5]

    for topic in self._topics: 
        # publish the message on the correct topic    
        if str(topic.name) == str("/" + parent_name + "/" + component_instance.blender_obj.name): 
            topic.publish(twist)
