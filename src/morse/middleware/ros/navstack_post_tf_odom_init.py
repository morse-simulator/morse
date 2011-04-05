import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('nav_msgs'); roslib.load_manifest('rosgraph_msgs')
import rospy
import std_msgs
from nav_msgs.msg import Odometry
import GameLogic
import math
#if GameLogic.pythonVersion < 3:
#    import Mathutils as mathutils
#else:
#    import mathutils
import mathutils

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    # Add the new method to the component
    component_instance.output_functions.append(function)
    self._topics.append(rospy.Publisher("/odom_init", Odometry))    

    # The following parts are used to calculate linear and angular velocities based on ground truth for fake localization
    # Variables to store the previous position
    component_instance.ppx = 0.0
    component_instance.ppy = 0.0
    component_instance.ppz = 0.0
    # Variables to store the previous velocity
    component_instance.pvx = 0.0
    component_instance.pvy = 0.0
    component_instance.pvz = 0.0
    # Variables to store previous angles
    component_instance.proll = 0.0
    component_instance.ppitch = 0.0
    component_instance.pyaw = 0.0
    # Variables to store previous angle-velocities
    component_instance.pvroll = 0.0
    component_instance.pvpitch = 0.0
    component_instance.pvyaw = 0.0
    # Start at 0 for odometry frame
    component_instance.initialpose = [component_instance.position_3d.x, component_instance.position_3d.y, component_instance.position_3d.z]
    component_instance.initialroll = component_instance.position_3d.roll
    component_instance.initialpitch = component_instance.position_3d.pitch
    component_instance.initialyaw = component_instance.position_3d.yaw
    component_instance.p = [0.0, 0.0, 0.0]
    component_instance.v = [0.0, 0.0, 0.0]            # Velocity
    component_instance.pv = [0.0, 0.0, 0.0]           # Previous Velocity
    component_instance.a = [0.0, 0.0, 0.0]            # Acceleration
    component_instance.o = [component_instance.position_3d.roll, component_instance.position_3d.pitch, component_instance.position_3d.yaw]
    component_instance.vangles = [0.0, 0.0, 0.0]      # Angle velocity
    component_instance.pvangles = [0.0, 0.0, 0.0]     # Previous angle velocities
        
    # Tick rate is the real measure of time in Blender.
    # By default it is set to 60, regardles of the FPS
    # If logic tick rate is 60, then: 1 second = 60 ticks
    component_instance.ticks = GameLogic.getLogicTicRate()

    component_instance.local_data['distance'] = 0.0
    component_instance.local_data['velocity'] = [0.0, 0.0, 0.0]
    # Note: acceleration is not used at the moment
    component_instance.local_data['acceleration'] = [0.0, 0.0, 0.0]
    component_instance.local_data['angle_velocities'] = [0.0, 0.0, 0.0]
    self.lasttime = rospy.Time.now()
    
    print ('######## ODOMETRY PUBLISHER INITIALIZED ########')
    
def post_odom_init(self, component_instance):
    """ Publish the data on the rostopic
    Here we calculate Odometry information based on the truepose of the Blender-object
	"""
    parent_name = component_instance.robot_parent.blender_obj.name
    odometry = Odometry()
    
    # Since we want odom to be 0 in position and orientation at the robots starting-point, we substract the initial values before we write the odom messages to local data    
    odometry.pose.pose.position.x = component_instance.initialpose[0]
    odometry.pose.pose.position.y = component_instance.initialpose[1]
    odometry.pose.pose.position.z = component_instance.initialpose[2]
    
    yaw = component_instance.initialyaw
    pitch = component_instance.initialpitch
    roll = component_instance.initialroll
    euler = mathutils.Euler((roll, pitch, yaw))
    quaternion = euler.to_quat()
    odometry.pose.pose.orientation.w = quaternion.w
    odometry.pose.pose.orientation.x = quaternion.x
    odometry.pose.pose.orientation.y = quaternion.y
    odometry.pose.pose.orientation.z = quaternion.z
        
    for topic in self._topics: 
        message = odometry
        # publish the message on the correct topic    
        if str(topic.name) == str("/odom_init"):
            topic.publish(odometry)
