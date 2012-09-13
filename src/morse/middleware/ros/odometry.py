import logging; logger = logging.getLogger("morse." + __name__)

import copy
import mathutils
import math

import roslib; roslib.load_manifest('rospy'); roslib.load_manifest('nav_msgs'); roslib.load_manifest('geometry_msgs')

from morse.middleware.ros.tfMessage import tfMessage

import rospy
from geometry_msgs.msg import Vector3, Quaternion, Pose, Twist, TransformStamped
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
    if mw_data[1] == "post_odometry":
        self._topics.append(rospy.Publisher("/" + parent_name + "/" + component_name, Odometry))
    elif mw_data[1] == "post_tf":
        logger.info("posting Odometry information via TF")

    # default frame_ids
    blender_frame_id = "/morse_world"
    odom_frame_id = "/odom"
    child_frame_id = "/base_footprint"

    # Extract the Middleware parameters
    # additional parameter should be a dict
    try:
        blender_frame_id = mw_data[3].get("blender_frame_id", blender_frame_id)
        odom_frame_id = mw_data[3].get("odom_frame_id", odom_frame_id)
        child_frame_id = mw_data[3].get("child_frame_id", child_frame_id)
    except:
        pass

    # create a new dictionary for this sensor if necessary
    if component_name not in self._properties:
        self._properties[component_name] = {}
    # store the frame ids in the dict
    self._properties[component_name]['blender_frame_id'] = blender_frame_id
    self._properties[component_name]['odom_frame_id'] = odom_frame_id
    self._properties[component_name]['child_frame_id'] = child_frame_id

    # store the initial pose
    pos = component_instance.robot_parent.blender_obj.worldPosition
    quat = component_instance.robot_parent.blender_obj.worldOrientation.to_quaternion()
    self._inital_translation = Vector3(x=pos.x, y=pos.y, z=pos.z)
    self._inital_rotation = Quaternion(x=quat.x, y=quat.y, z=quat.z, w=quat.w)

    self.pub_tf = rospy.Publisher("/tf", tfMessage)

    self._seq = 0

    logger.info("Initialized the ROS odometry sensor with blender_frame_id '%s'" \
                ", odom_frame_id '%s' and child_frame_id '%s'",
                blender_frame_id, odom_frame_id, child_frame_id)

def sendTransform(self, translation, rotation, time, child, parent):
    """
    :param translation: the translation of the transformtion as geometry_msgs/Vector3
    :param rotation: the rotation of the transformation as a geometry_msgs/Quaternion
    :param time: the time of the transformation, as a rospy.Time()
    :param child: child frame in tf, string
    :param parent: parent frame in tf, string
    
    Broadcast the transformation from tf frame child to parent on ROS topic ``"/tf"``.
    """

    t = TransformStamped()
    t.header.frame_id = parent
    t.header.stamp = time
    t.child_frame_id = child
    t.transform.translation = translation
    t.transform.rotation = rotation

    tfm = tfMessage([t])
    self.pub_tf.publish(tfm)

def get_orientation(self, component_instance):
    """ Get the orientation from the local_data and return a quaternion """
    euler = mathutils.Euler((component_instance.local_data['roll'],
                             component_instance.local_data['pitch'],
                             component_instance.local_data['yaw']))
    quaternion = euler.to_quaternion()
    return quaternion

def get_translation(self, component_instance):
    """ Get the position from the local_data and return a ROS Vector3 """
    translation = Vector3()
    translation.x = component_instance.local_data['x']
    translation.y = component_instance.local_data['y']
    translation.z = component_instance.local_data['z']
    return translation

def get_pose(self, component_instance):
    """ Get the pose from the local_data and return a ROS Pose """
    pose = Pose()
    pose.position = get_translation(self, component_instance)
    pose.orientation = get_orientation(self, component_instance)
    return pose

def get_twist(self, component_instance):
    """ Get the twist from the local_data and return a ROS Twist """
    twist = Twist()
    twist.linear.x = component_instance.local_data['vx']
    twist.linear.y = component_instance.local_data['vy']
    twist.linear.z = component_instance.local_data['vz']
    twist.angular.x = component_instance.local_data['wx']
    twist.angular.y = component_instance.local_data['wy']
    twist.angular.z = component_instance.local_data['wz']
    return twist

def post_tf(self, component_instance):
    """ Publish the odometry estimate and initial pose via ROS tf """
    component_name = component_instance.blender_obj.name
    blender_frame_id = self._properties[component_name]['blender_frame_id']
    odom_frame_id = self._properties[component_name]['odom_frame_id']
    child_frame_id = self._properties[component_name]['child_frame_id']

    time = rospy.Time.now()

    # send current odometry transform
    sendTransform(self, get_translation(self, component_instance),
                  get_orientation(self, component_instance),
                  time,
                  child_frame_id,
                  odom_frame_id)

    # send initial transformation from blender to odom frame
    sendTransform(self, self._inital_translation,
                  self._inital_rotation,
                  time,
                  odom_frame_id,
                  blender_frame_id)

def post_odometry(self, component_instance):
    """ Publish the data of the odometry sensor as a ROS-Odometry message
    """

    parent_name = component_instance.robot_parent.blender_obj.name
    component_name = component_instance.blender_obj.name
    blender_frame_id = self._properties[component_name]['blender_frame_id']
    odom_frame_id = self._properties[component_name]['odom_frame_id']
    child_frame_id = self._properties[component_name]['child_frame_id']

    time = rospy.Time.now()

    odometry = Odometry()
    odometry.header.seq = self._seq
    odometry.header.stamp = time
    odometry.header.frame_id = odom_frame_id
    odometry.child_frame_id = child_frame_id

    # fill pose and twist
    odometry.pose.pose = get_pose(self, component_instance)
    odometry.twist.twist = get_twist(self, component_instance)

    for topic in self._topics:
        # publish the message on the correct w
        if str(topic.name) == str("/" + parent_name + "/" + component_instance.blender_obj.name):
            topic.publish(odometry)

    # send current odometry transform
    sendTransform(self, get_translation(self, component_instance),
                  get_orientation(self, component_instance),
                  time,
                  child_frame_id,
                  odom_frame_id)

    self._seq += 1
