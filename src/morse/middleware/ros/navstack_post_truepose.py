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
    self._topics.append(rospy.Publisher("/base_pose_ground_truth", Odometry))    

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
    # Make a new reference to the sensor position
    component_instance.p = [component_instance.position_3d.x, component_instance.position_3d.y, component_instance.position_3d.z]
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
    #component_instance.local_data['acceleration'] = [0.0, 0.0, 0.0]
    component_instance.local_data['angle_velocities'] = [0.0, 0.0, 0.0]
    
    print ('######## ODOMETRY PUBLISHER INITIALIZED ########')
    

#Note: posting 2D Laserscans is still experimental and maybe the ranges are not correctly calculated!
def post_truepose(self, component_instance):
    """ Publish the data on the rostopic
	"""
    parent_name = component_instance.robot_parent.blender_obj.name
    odometry = Odometry()
        
    odometry.pose.pose.position.x = component_instance.position_3d.x
    odometry.pose.pose.position.y = component_instance.position_3d.y
    odometry.pose.pose.position.z = component_instance.position_3d.z
    euler = mathutils.Euler((component_instance.position_3d.roll, component_instance.position_3d.pitch, component_instance.position_3d.yaw))
    quaternion = euler.to_quat()
    odometry.pose.pose.orientation.w = quaternion.w
    odometry.pose.pose.orientation.x = quaternion.x
    odometry.pose.pose.orientation.y = quaternion.y
    odometry.pose.pose.orientation.z = quaternion.z
    
    # Compute the difference in positions with the previous loop
    component_instance.p = [component_instance.position_3d.x, component_instance.position_3d.y, component_instance.position_3d.z]
    dx = component_instance.p[0] - component_instance.ppx
    dy = component_instance.p[1] - component_instance.ppy
    dz = component_instance.p[2] - component_instance.ppz
    distance = math.sqrt(dx**2 + dy**2 + dz**2)
    #print ("DISTANCE: %.4f" % component_instance.local_data['distance'])

    # Compute the difference in angles with the previous loop
    component_instance.o = [component_instance.position_3d.roll, component_instance.position_3d.pitch, component_instance.position_3d.yaw]
    droll = component_instance.o[0] - component_instance.proll
    dpitch = component_instance.o[1] - component_instance.ppitch
    dyaw = component_instance.o[2] - component_instance.pyaw

    # Store the position in this instant
    component_instance.ppx = component_instance.p[0]
    component_instance.ppy = component_instance.p[1]
    component_instance.ppz = component_instance.p[2]
        
    # Store the rotation in this instant
    component_instance.proll = component_instance.o[0]
    component_instance.ppitch = component_instance.o[1]
    component_instance.pyaw = component_instance.o[2]

    # Scale the speeds to the time used by Blender
    component_instance.v[0] = dx * component_instance.ticks
    component_instance.v[1] = dy * component_instance.ticks
    component_instance.v[2] = dz * component_instance.ticks
    #print ("SPEED: (%.4f, %.4f, %.4f)" % (component_instance.v[0], component_instance.v[1], component_instance.v[2]))

    # Scale the speeds to the time used by Blender
    component_instance.vangles[0] = droll * component_instance.ticks
    component_instance.vangles[1] = dpitch * component_instance.ticks
    component_instance.vangles[2] = dyaw * component_instance.ticks
    #print ("Angle SPEED: (%.4f, %.4f, %.4f)" % (component_instance.vangles[0], component_instance.vangles[1], component_instance.vangles[2]))

    component_instance.a[0] = (component_instance.v[0] - component_instance.pvx) * component_instance.ticks
    component_instance.a[1] = (component_instance.v[1] - component_instance.pvy) * component_instance.ticks
    component_instance.a[2] = (component_instance.v[2] - component_instance.pvz) * component_instance.ticks
    #print ("ACCELERATION: (%.4f, %.4f, %.4f)" % (component_instance.a[0], component_instance.a[1], component_instance.a[2]))
    
    # Update the data for the velocity
    component_instance.pvx = component_instance.v[0]
    component_instance.pvy = component_instance.v[1]
    component_instance.pvz = component_instance.v[2]
        
    component_instance.pvroll = component_instance.vangles[0]
    component_instance.pvpitch = component_instance.vangles[1]
    component_instance.pvyaw = component_instance.vangles[2]

    # Store the important data
    component_instance.local_data['velocity'] = component_instance.v
    component_instance.local_data['acceleration'] = component_instance.a
    component_instance.local_data['angle_velocities'] = component_instance.vangles

    odometry.twist.twist.linear.x =  component_instance.local_data['velocity'][0]
    odometry.twist.twist.linear.y =  component_instance.local_data['velocity'][1]
    odometry.twist.twist.linear.z =  component_instance.local_data['velocity'][2]
    
    odometry.twist.twist.angular.x =  component_instance.local_data['angle_velocities'][0]
    odometry.twist.twist.angular.y =  component_instance.local_data['angle_velocities'][1]
    odometry.twist.twist.angular.z =  component_instance.local_data['angle_velocities'][2]
    
    odometry.header.stamp = rospy.Time.now()
    # For truepose, baseframe is map
    odometry.header.frame_id = "map"
    
    for topic in self._topics: 
        message = odometry
        # publish the message on the correct topic    
        if str(topic.name) == str("/base_pose_ground_truth"):
            topic.publish(odometry)
