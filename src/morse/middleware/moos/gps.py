import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.moos import MOOSNotifier

class GPSNotifier(MOOSNotifier):
    """ Notify GPS """

    def default(self, ci='unused'):
        self.notify('MORSE_GPS_E_X', self.data['x'])
        self.notify('MORSE_GPS_N_Y', self.data['y'])
        self.notify('MORSE_GPS_ALT_Z', self.data['z'])

class GPSRawNotifier(MOOSNotifier):
    """ Notify Raw GPS """

    def default(self, ci='unused'):
        self.notify('MORSE_GPS_E_X', self.data['x'])
        self.notify('MORSE_GPS_N_Y', self.data['y'])
        self.notify('MORSE_GPS_ALT_Z', self.data['z'])
        self.notify('MORSE_GPS_LAT', self.data['latitude'])
        self.notify('MORSE_GPS_LON', self.data['longitude'])
