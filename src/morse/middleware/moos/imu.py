import pymoos.MOOSCommClient
import morse.core.middleware
import GameLogic
import mathutils

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
    print('######## POSE-SENSOR INITIALIZED ########')

def post_imu(self, component_instance):
    """ Publish the data of the Odometry-sensor as a ROS-Pose message
    """
    curTime=GameLogic.current_time
    parent_name = component_instance.robot_parent.blender_obj.name

    vel=component_instance.local_data['velocity']
    acc=component_instance.local_data['acceleration']
    
    # post angular rates
    self.m.Notify('zGyroX',vel[3],curTime)
    self.m.Notify('zGyroY',vel[4],curTime)
    self.m.Notify('zGyroZ',vel[5],curTime)

    # post accelerations
    self.m.Notify('zAccelX',acc[0],curTime)
    self.m.Notify('zAccelY',acc[1],curTime)
    self.m.Notify('zAccelZ',acc[2],curTime)
   
