import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.moos import MOOSNotifier
try:
    from pymoos import pymoos
except:
    import pymoos

class DVLNotifier(MOOSNotifier):
    """ Notify DVL """

    def default(self, ci = 'unused'):
        logger.debug('DVLNotifier is publising!')
    
        ts = self.data['timestamp']

        self.notify('MORSE_DVL_WORLD_VEL_X', self.data['Wx'],ts)
        self.notify('MORSE_DVL_WORLD_VEL_Y', self.data['Wy'],ts)
        self.notify('MORSE_DVL_WORLD_VEL_Z', self.data['Wz'],ts)
        self.notify('MORSE_DVL_BODY_VEL_X',  self.data['Bx'],ts)
        self.notify('MORSE_DVL_BODY_VEL_Y',  self.data['By'],ts)
        self.notify('MORSE_DVL_BODY_VEL_Z',  self.data['Bz'],ts)
        self.notify('MORSE_DVL_HEADING',     self.data['heading'],ts)
        self.notify('MORSE_DVL_ALTITUDE',    self.data['z'],ts)
        self.notify('MORSE_DVL_DEPTH',       self.data['depth'],ts)
