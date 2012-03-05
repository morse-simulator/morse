#!/usr/bin/env python  
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('tf')
roslib.load_manifest('roscpp')
roslib.load_manifest('nav_msgs')
roslib.load_manifest('rosgraph_msgs')
roslib.load_manifest('geometry_msgs')
import rospy
import time
import math
import tf
import geometry_msgs
import std_msgs
import nav_msgs
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3Stamped

# Global variables to store the robot starting position in the /map frame for building the /odom frame
br3 = tf.TransformBroadcaster()
odom_init = 0
odom_pose_x = 0
odom_pose_y = 0
odom_pose_z = 0
odom_quat_x = 0
odom_quat_y = 0
odom_quat_z = 0
odom_quat_w = 0

# Global variables for odometry message
pub = rospy.Publisher('/odom', Odometry)
                     
def handle_odom_tf(msg, robotname):
    
    global br3
    global odom_init
    global odom_pose_x
    global odom_pose_y
    global odom_pose_z
    global odom_quat_x
    global odom_quat_y
    global odom_quat_z
    global odom_quat_w

    global odom_last_pose_x
    global last_time

    # To obtain the initial point of the odom-frame, the global coordinates are set with the first message
    if odom_init == 0:
        odom_pose_x = msg.pose.pose.position.x
        odom_pose_y = msg.pose.pose.position.y
        odom_pose_z = msg.pose.pose.position.z
        odom_quat_x = msg.pose.pose.orientation.x
        odom_quat_y = msg.pose.pose.orientation.y
        odom_quat_z = msg.pose.pose.orientation.z
        odom_quat_w = msg.pose.pose.orientation.w
        odom_init = 1

    # publish the odom init
    br3.sendTransform((odom_pose_x, odom_pose_y, odom_pose_z),
                    (odom_quat_x, odom_quat_y, odom_quat_z, odom_quat_w),
                    rospy.Time.now(),
                    "/odom",
                    "/map")
    # transformation bewteen map and robot frame
    br3.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     "/base_footprint",
                     "/map")

def handle_odom_msg(msg, robotname):

    global pub
    odometry = Odometry()

    # We are only interested in the velocity values since we use fake localization
    odometry.twist.twist.linear.x = math.sqrt(msg.linear.x**2 + msg.linear.y**2) #v_omega controller => no sidewarts movement
    odometry.twist.twist.linear.y = 0
    odometry.twist.twist.linear.z = msg.linear.z
    odometry.twist.twist.angular.x = msg.angular.x
    odometry.twist.twist.angular.y = msg.angular.y
    odometry.twist.twist.angular.z = msg.angular.z

    odometry.header.stamp = rospy.Time.now()
    odometry.header.frame_id = "/odom"
    odometry.child_frame_id = "/base_link"

    pub.publish(odometry) 

if __name__ == '__main__':
    rospy.init_node('morse_tf_broadcaster')
    
    # Initialize odom-frame with robot starting-position
    rospy.Subscriber('/pr2/Pose_sensor',
                     nav_msgs.msg.Odometry,
                     handle_odom_tf,
                     '/pr2/Pose_sensor')

    # Handle speed-information using IMU sensor of Jido
    rospy.Subscriber('/pr2/IMU',
                     geometry_msgs.msg.Twist,
                     handle_odom_msg,
                     '/pr2/Pose_sensor')

    rospy.spin()
