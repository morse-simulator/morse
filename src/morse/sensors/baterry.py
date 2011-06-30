import GameLogic
import time
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
        self._time = time.clock()

        print ('######## BATTERY INITIALIZED ########')

    def default_action(self):
        """ Main function of this component. """
        newtime = time.clock()
        c = self.local_data['charge']
        dt = newtime - self._time
        v = c - (dt * self.blender_obj['DischargingRate'])
        self._time = newtime

        # Store the data acquired by this sensor that could be sent
        #  via a middleware.
        self.local_data['charge'] = float(v)

