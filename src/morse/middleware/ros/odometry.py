import logging; logger = logging.getLogger("morse." + __name__)
import mathutils
import roslib; roslib.load_manifest('rospy'); roslib.load_manifest('nav_msgs'); roslib.load_manifest('geometry_msgs')
import rospy
from geometry_msgs.msg import Vector3, Quaternion, Pose, Twist, TransformStamped
from nav_msgs.msg import Odometry
from morse.middleware.ros.tfMessage import tfMessage

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    # default frame_ids
    blender_frame_id = "/morse_world"
    odom_frame_id = "/odom"
    child_frame_id = "/base_footprint"
    # Extract the Middleware parameters. additional parameter should be a dict
    try:
        blender_frame_id = mw_data[3].get("blender_frame_id", blender_frame_id)
        odom_frame_id = mw_data[3].get("odom_frame_id", odom_frame_id)
        child_frame_id = mw_data[3].get("child_frame_id", child_frame_id)
    except:
        pass

    self.set_property(component_instance, 'blender_frame_id', blender_frame_id)
    self.set_property(component_instance, 'odom_frame_id', odom_frame_id)
    self.set_property(component_instance, 'child_frame_id', child_frame_id)

    # store the initial pose
    self.set_property(component_instance, 'inital_translation',
                      component_instance.robot_parent.blender_obj.worldPosition)
    self.set_property(component_instance, 'inital_rotation',
                      component_instance.robot_parent.blender_obj.worldOrientation.to_quaternion())

    if mw_data[1] == "post_odometry":
        self.register_publisher(component_instance, function, Odometry)
    elif mw_data[1] == "post_tf":
        logger.info("posting Odometry information via TF")
        component_instance.output_functions.append(function)

    self.register_publisher_tf()

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

    self.publish_topic(tfm, "/tf")

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
    blender_frame_id = self.get_property(component_instance, 'blender_frame_id')
    odom_frame_id = self.get_property(component_instance, 'odom_frame_id')
    child_frame_id = self.get_property(component_instance, 'child_frame_id')

    time = rospy.Time.now()

    # send current odometry transform
    sendTransform(self, get_translation(self, component_instance),
                  get_orientation(self, component_instance),
                  time, child_frame_id, odom_frame_id)

    # send initial transformation from blender to odom frame
    sendTransform(self,
                  self.get_property(component_instance, 'inital_translation'),
                  self.get_property(component_instance, 'inital_rotation'),
                  time, odom_frame_id, blender_frame_id)

def post_odometry(self, component_instance):
    """ Publish the data of the odometry sensor as a ROS Odometry message
    """
    odometry = Odometry()
    odometry.header = self.get_ros_header(component_instance)
    odometry.header.frame_id = self.get_property(component_instance, 'odom_frame_id')
    odometry.child_frame_id = self.get_property(component_instance, 'child_frame_id')

    # fill pose and twist
    odometry.pose.pose = get_pose(self, component_instance)
    odometry.twist.twist = get_twist(self, component_instance)

    self.publish(odometry, component_instance)

    # send current odometry transform
    sendTransform(self, get_translation(self, component_instance),
                  get_orientation(self, component_instance),
                  odometry.header.stamp,
                  odometry.child_frame_id,
                  odometry.header.frame_id)
