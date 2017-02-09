import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.moos import MOOSNotifier

class GPSNotifier(MOOSNotifier):
    """ Notify GPS """

    def default(self, ci='unused'):
        self.notify('MORSE_GPS_E_X', self.data['x'])
        self.notify('MORSE_GPS_N_Y', self.data['y'])
        self.notify('MORSE_GPS_ALT_Z', self.data['z'])
