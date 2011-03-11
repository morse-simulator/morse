import GameLogic
import morse.helpers.sensor

class MotionSensorClass(morse.helpers.sensor.MorseSensorClass):
    """ Class definition for the gyroscope sensor.
        Sub class of Morse_Object. """

    def __init__(self, obj, parent=None):
        """ Constructor method.
            Receives the reference to the Blender object.
            The second parameter should be the name of the object's parent. """
        print ("######## Motion Sensor '%s' INITIALIZING ########" % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self.local_data['status'] = parent.move_status

        print ('######## Motion Sensor INITIALIZED ########')


    def default_action(self):
        """ Main function of this component. """
        parent = self.robot_parent

        # Store the data acquired by this sensor that could be sent
        #  via a middleware.
        self.local_data['status'] = parent.move_status
