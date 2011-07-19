import GameLogic
import morse.core.actuator

class SteerForceActuatorClass(morse.core.actuator.MorseActuatorClass):
    """ Motion controller using engine force and steer angle speeds

    This class will read engine force and steer angle (steer, force)
    as input from an external middleware, and then apply them
    to the parent robot.
    """

    def __init__(self, obj, parent=None):
        print ('######## STEER_FORCE CONTROL INITIALIZATION ########')
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self.local_data['steer'] = 0.0
        self.local_data['force'] = 0.0
        self.local_data['brake'] = 0.0
        #self.local_data['steer'] = 0.0
        #self.local_data['force'] = 0.0
        
        # Choose the type of function to move the object
        #self._type = 'Velocity'
        #self._type = 'Position'

        print ('######## CONTROL INITIALIZED ########')



    def default_action(self):
        """ Apply (steer, force) to the parent robot. """
        # Get the Blender object of the parent robot
        parent = self.robot_parent
        
        #Update the steering value for these wheels:
        #The number the end represents the wheel 'number' in the 
        #order they were created above.  Front wheels #0 and #1.
        #Rear wheels #2 and #3.
        parent.vehicle.setSteeringValue(self.local_data['steer'],0)
        parent.vehicle.setSteeringValue(self.local_data['steer'],1)

        #Update the Force (speed) for these wheels:
        parent.vehicle.applyEngineForce(self.local_data['force']*.4,0)
        parent.vehicle.applyEngineForce(self.local_data['force']*.4,1)
        parent.vehicle.applyEngineForce(self.local_data['force']*.4,2)
        parent.vehicle.applyEngineForce(self.local_data['force'] *.4,3)

        #Brakes:
        #Applies the braking force to each wheel listed:
        #chassis = owner of the controller, in my example, it's the object
        #labeled 'car'
        #['brakes'] = the game property value for the car labeled 'brakes'
        #Default value is 0:
        #Pressing the space bar assigns it a value of 3 for braking
        parent.vehicle.applyBraking(self.local_data['brake']*.1,0)
        parent.vehicle.applyBraking(self.local_data['brake']*.1,1)
        parent.vehicle.applyBraking(self.local_data['brake']*1.3,2)
        parent.vehicle.applyBraking(self.local_data['brake']*1.3,3)

