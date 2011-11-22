import logging; logger = logging.getLogger("morse." + __name__)
import GameLogic
import morse.core.sensor

class PTUPostureClass(morse.core.sensor.MorseSensorClass):
    """ Reader for the PTU posture sensor """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)
        
        self._ptu_found = False

        self.local_data['pan'] = 0.0
        self.local_data['tilt'] = 0.0
        logger.info('Component initialized')


    def find_ptu(self):
        # Check if I have a child called 'PTU'
        for child in self.blender_obj.childrenRecursive:
            if self.blender_obj['PTUname'] in str(child):
                logger.debug("Found a PTU called: %s" % child.name)
                #self._ptu_obj = child
                try:
                    self._ptu_obj = GameLogic.componentDict[child.name]
                    self._ptu_found = True
                except:
                    return
 

    def default_action(self):
        """ Read the rotation of the platine unit """
        # Find the actual PTU unit as my child
        if self._ptu_found == False:
            self.find_ptu()
            return

        # Update the postition of the base platforms
        current_pan, current_tilt = self._ptu_obj.get_pan_tilt()
        logger.debug("Platine: pan=%.4f, tilt=%.4f" % (current_pan, current_tilt))
        
        # Store the data acquired by this sensor that could be sent
        #  via a middleware.
        self.local_data['pan'] = float(current_pan)
        self.local_data['tilt'] = float(current_tilt)
