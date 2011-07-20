import pymoos.MOOSCommClient
import morse.core.middleware
import GameLogic
#if GameLogic.pythonVersion < 3:
#    import Mathutils as mathutils
#else:
import mathutils

class MOOSClass(morse.core.middleware.MorseMiddlewareClass):
    """ Handle communication between Blender and MOOS."""
      
    def __init__(self, obj, parent=None):
        """ Initialize the MOOS app"""
        super(self.__class__,self).__init__(obj, parent)
        print ("################# Initializing MOOS middleware ###################")
        self.m = pymoos.MOOSCommClient.MOOSApp()
        #self.m.SetOnConnectCallBack( self.m.DoRegistrations )
        #self.m.SetOnMailCallBack( self.m.MailCallback )
        
        print("%s" % self.m.GetLocalIPAddress())

        fundamental_frequency = 10 # [Hz]
        self.m.Run( "127.0.0.1", 9000, "MORSE_SIM", fundamental_frequency) 
        print ("################# MOOS middleware initialized ####################")
        
        
    def __del__(self):
        """ Kill the morse MOOS app."""
        self.m.Close();
        print("Shutting down MOOS app...")
   
      
    def register_component(self, component_name, component_instance, mw_data):
        """ Generate a new topic to publish the data

        The name of the topic is composed of the robot and sensor names.
        Only useful for sensors.
        """

		# register for control variables from the database
        # tells 
        self.m.Register("cVelocity")
        self.m.Register("cYawRate")
		
        print("========== Registering component =================")
        parent_name = component_instance.robot_parent.blender_obj.name

        # Extract the information for this middleware
        # This will be tailored for each middleware according to its needs
        # This is specified in the component_config.py in Blender: [mw_data[0], mw_data[1]]
        function_name = mw_data[1]
        print (" ######################## %s"%parent_name)
        print (" ######################## %s"%component_name )
        
        # make sure the handler function exists
        function = self._check_function_exists(function_name)
        
        # The function exists within this class,
        #  so it can be directly assigned to the instance
        if function != None:
            
            # Add data publish functions to output_functions
            if function_name == "post_message":
                component_instance.output_functions.append(function)
                # Generate one publisher and one topic for each component that is a sensor and uses post_message 
                #self._topics.append(rospy.Publisher(parent_name + "/" + component_name, String))
        
            # Read Strings from a rostopic    
            elif function_name == "read_message":
                component_instance.input_functions.append(function)
                #func = getattr(self, "callback")
                #self._topics.append(rospy.Subscriber(parent_name + "/" + component_name, String, func, component_instance))
       
            else:
                #Add external module 
                #self._add_method(mw_data, component_instance)
                pass
        else:
            # If there is no such function in this module,
            #  try importing from another one
            try:
                # Insert the method in this class
                function = self._add_method(mw_data, component_instance)
            except IndexError as detail:
                print ("ERROR: Method '%s' is not known, and no external module has been specified. Check the 'component_config.py' file for typos" % function_name)
                return
                
        print("Component registered")

    # Post string messages
    def post_message(self, component_instance):
        """ Publish the data to the MOOS database
        """
        #print("Posting message to the MOOS database.")
        parent_name = component_instance.robot_parent.blender_obj.name

        #iterate through all objects of the component_instance and post the data
        for variable, data in component_instance.local_data.items():
            #print(parent_name+"_"+component_instance.blender_obj.name+"_"+variable)
            #print(str(data))
            #print(type(data))
            self.m.Notify(parent_name+"_"+component_instance.blender_obj.name+"_"+variable,str(data),GameLogic.current_time)

                            
    # NOTE: This is a dummy function that is executed for every actuator. Since ROS uses the concept of callbacks, it does nothing ...    
    def read_message(self, component_instance):
        """ read a command message from the database and send to the simulator???"""
        #print("Read message called.")
        current_time = pymoos.MOOSCommClient.MOOSTime()
        # get latest mail from the MOOS comm client
        messages = self.m.FetchRecentMail()
        
        # look for command messages: cYawRate and cVelocity
        for message in messages:
            if (message.GetKey()=="cVelocity") and (message.IsDouble()):
                component_instance.local_data['v'] =message.GetDouble() # command linear velocity [m/s]
            elif  (message.GetKey()=="cYawRate") and (message.IsDouble()):
                component_instance.local_data['w']=message.GetDouble() # command angular velocity [m/s]
            elif  (message.GetKey()=="cSteer") and (message.IsDouble()):
                component_instance.local_data['steer']=message.GetDouble() # command steer angle [deg]
            elif  (message.GetKey()=="cThrottle") and (message.IsDouble()):
                component_instance.local_data['force']=message.GetDouble() # command engine force
            elif  (message.GetKey()=="cBrake") and (message.IsDouble()):
                component_instance.local_data['brake']=message.GetDouble() # command angular velocity [m/s]
