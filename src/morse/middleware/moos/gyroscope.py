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
    print('######## Gyroscope-SENSOR INITIALIZED ########')

def post_gyroscope(self, component_instance):
    """ Publish the data of the Odometry-sensor as a ROS-Pose message
    """
    curTime=pymoos.MOOSCommClient.MOOSTime()
    
    self.m.Notify('zYaw',component_instance.local_data['yaw'],curTime)
    self.m.Notify('zRoll',component_instance.local_data['roll'],curTime)
    self.m.Notify('zPitch',component_instance.local_data['pitch'],curTime)