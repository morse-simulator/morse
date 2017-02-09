import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.moos import MOOSSubscriber

class MotionReader(MOOSSubscriber):
    """ Read motion commands and update local data. """

    def initialize(self):
        MOOSSubscriber.initialize(self)
        # register for control variables from the database
        self.register_message_to_queue("MORSE_MOTION_VELOCITY",
                    'motion_queue', self.on_motion_msgs)
        self.register_message_to_queue("MORSE_MOTION_YAWRATE",
                    'motion_queue', self.on_motion_msgs)
        self.register_message_to_queue("MORSE_MOTION_STEER",
                    'motion_queue', self.on_motion_msgs)
        self.register_message_to_queue("MORSE_MOTION_THROTTLE",
                    'motion_queue', self.on_motion_msgs)
        self.register_message_to_queue("MORSE_MOTION_BRAKE",
                    'motion_queue', self.on_motion_msgs)

    def on_motion_msgs(self, msg):
        if (msg.key() == "MORSE_MOTION_VELOCITY") and (msg.is_double()):
            self.data['v'] = msg.double() # command linear velocity [m/s]
        elif  (msg.key() == "MORSE_MOTION_YAWRATE") and (msg.is_double()):
            self.data['w'] = msg.double() # command angular velocity [m/s]
        elif  (msg.key() == "MORSE_MOTION_STEER") and (msg.is_double()):
            self.data['steer'] = msg.double() # command steer angle [deg]
        elif  (msg.key() == "MORSE_MOTION_THROTTLE") and (msg.is_double()):
            self.data['force'] = msg.double() # command engine force
        elif  (msg.key() == "MORSE_MOTION_BRAKE") and (msg.is_double()):
            self.data['brake'] = msg.double() # command angular velocity [m/s]
        self._new_messages = True
        return True

    def update_morse_data(self):
        logger.debug('MotionReader.update_morse_data() called.')
