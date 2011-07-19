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

def post_pose(self, component_instance):
    """ Publish the data of the Odometry-sensor as a ROS-Pose message
    """
    curTime=GameLogic.current_time
    parent_name = component_instance.robot_parent.blender_obj.name
    
    self.m.Notify('simEast',component_instance.local_data['x'],curTime)
    self.m.Notify('simNorth',component_instance.local_data['y'],curTime)
    self.m.Notify('simHeight',component_instance.local_data['z'],curTime)
    self.m.Notify('simYaw',component_instance.local_data['yaw'],curTime)
    self.m.Notify('simRoll',component_instance.local_data['roll'],curTime)
    self.m.Notify('simPitch',component_instance.local_data['pitch'],curTime)

    #~ for variable, data in component_instance.local_data.items():
        #~ if (variable=='yaw'):
            #~ postName='simYaw'
        #~ elif (variable=='roll'):
            #~ postName='simRoll'
        #~ elif (variable=='pitch'):
            #~ postName='simPitch'
        #~ elif (variable=='x'):
            #~ postName='simEast'
        #~ elif (variable=='y'):
            #~ postName='simNorth'
        #~ elif (variable=='z'):
            #~ postName='simHeight'
        #~ else:
            #~ postName=''

        #~ if (postName!=''):
            #~ self.m.Notify(postName,data,curTime)


   
