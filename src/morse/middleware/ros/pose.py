import logging; logger = logging.getLogger("morse." + __name__)
import math
import mathutils
import roslib; roslib.load_manifest('rospy'); roslib.load_manifest('geometry_msgs'); roslib.load_manifest('nav_msgs')
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from morse.middleware.ros.tfMessage import tfMessage

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
    if mw_data[1] == "post_pose":
        self._topics.append(rospy.Publisher(parent_name + "/" + component_name, PoseStamped))
    else:
        self._topics.append(rospy.Publisher(parent_name + "/" + component_name, Odometry))

    self.pub_tf = rospy.Publisher("/tf", tfMessage)

    self._seq = 0

    logger.info('Initialized the ROS pose sensor')

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
    """ Publish the data of the Pose sensor as a ROS-Odometry message for fake localization
    """
    parent_name = component_instance.robot_parent.blender_obj.name

    odometry = Odometry()
    odometry.header.stamp = rospy.Time.now()
    odometry.header.frame_id = "/map"
    odometry.child_frame_id = "/base_footprint"

    odometry.pose.pose.position.x = component_instance.local_data['x']
    odometry.pose.pose.position.y = component_instance.local_data['y']
    odometry.pose.pose.position.z = component_instance.local_data['z']
    euler = mathutils.Euler((component_instance.local_data['roll'],
                             component_instance.local_data['pitch'],
                             component_instance.local_data['yaw']))
    odometry.pose.pose.orientation = euler.to_quaternion()

    for topic in self._topics:
        # publish the message on the correct topic
        if str(topic.name) == str("/" + parent_name + "/" + component_instance.blender_obj.name):
            topic.publish(odometry)

def post_odometry_tf(self, component_instance):
    """ Publish the data of the Pose sensor as a ROS-Odometry message with tf
    """
    parent_name = component_instance.robot_parent.blender_obj.name
    current_time = rospy.Time.now()

    x = component_instance.local_data['x']
    y = component_instance.local_data['y']
    z = component_instance.local_data['z']
    euler = mathutils.Euler((component_instance.local_data['roll'],
                             component_instance.local_data['pitch'],
                             component_instance.local_data['yaw']))
    quaternion = euler.to_quaternion()

    odometry = Odometry()
    odometry.header.seq = self._seq
    odometry.header.stamp = current_time
    odometry.header.frame_id = "/odom"
    odometry.child_frame_id = "/base_footprint"

    # fill pose
    odometry.pose.pose.position.x = x
    odometry.pose.pose.position.y = y
    odometry.pose.pose.position.z = z
    odometry.pose.pose.orientation = quaternion

    for topic in self._topics:
        # publish the message on the correct topic
        if str(topic.name) == str("/" + parent_name + "/" + component_instance.blender_obj.name):
            topic.publish(odometry)

    # publish the transformation
    sendTransform(self, (x, y, z),
                  (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                  current_time,
                  "/base_footprint",
                  "/odom")

    self._seq += 1

def post_pose(self, component_instance):
    """ Publish the data of the Pose as a ROS-PoseStamped message
    """
    parent_name = component_instance.robot_parent.blender_obj.name
    poseStamped = PoseStamped()

    poseStamped.pose.position.x = component_instance.local_data['x']
    poseStamped.pose.position.y = component_instance.local_data['y']
    poseStamped.pose.position.z = component_instance.local_data['z']
    euler = mathutils.Euler((component_instance.local_data['roll'],
                             component_instance.local_data['pitch'],
                             component_instance.local_data['yaw']))
    poseStamped.pose.orientation = euler.to_quaternion()

    poseStamped.header.stamp = rospy.Time.now()

    # Default baseframe is map
    poseStamped.header.frame_id = "map"

    for topic in self._topics:
        message = poseStamped
        # publish the message on the correct topic
        if str(topic.name) == str("/" + parent_name + "/" + component_instance.blender_obj.name):
            topic.publish(poseStamped)
