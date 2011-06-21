import GameLogic
import morse.core.sensor

class BatteryClass(morse.core.sensor.MorseSensorClass):
    """ Class definition for the battery sensor.
        Sub class of Morse_Object. 

        DischargingRate: float in percent-per-seconds (game-property)
    """

    def __init__(self, obj, parent=None):
        """ Constructor method.
            Receives the reference to the Blender object.
            The second parameter should be the name of the object's parent. """
        print ("######## BATTERY '%s' INITIALIZING ########" % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self.local_data['charge'] = 100.0
        self.local_data['time'] = 0.0

        print ('######## BATTERY INITIALIZED ########')

    def default_action(self):
        """ Main function of this component. """
        c = self.local_data['charge']
        dt = GameLogic.current_time - self.local_data['time']
        v = c - (dt * self.blender_obj['DischargingRate'])
        self.local_data['time'] = GameLogic.current_time
        # TODO time.time() ?

        # Store the data acquired by this sensor that could be sent
        #  via a middleware.
        self.local_data['charge'] = float(v)

