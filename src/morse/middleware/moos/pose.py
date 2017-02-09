import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.moos import MOOSNotifier, MOOSSubscriber
from morse.core import blenderapi

class PoseNotifier(MOOSNotifier):
    """ Notify Pose """

    def default(self,  ci='unused'):
        # post the simulation time so that it can be synced to MOOSTime
        self.notify('MORSE_TIME',
                      blenderapi.persistantstorage().time.time)
        # post the robot position
        self.notify('MORSE_SIM_X', self.data['x'])
        self.notify('MORSE_SIM_Y', self.data['y'])
        self.notify('MORSE_SIM_Z', self.data['z'])
        self.notify('MORSE_SIM_YAW', self.data['yaw'])
        self.notify('MORSE_SIM_ROLL', self.data['roll'])
        self.notify('MORSE_SIM_PITCH', self.data['pitch'])

class PoseReader(MOOSSubscriber):
    """ Read pose commands and update local data. """

    def initialize(self):
        MOOSSubscriber.initialize(self)
        # register for position variables from the database
        self.register_message_to_queue("MORSE_SET_X",
                    'pose_queue', self.on_pose_msgs)
        self.register_message_to_queue("MORSE_SET_Y",
                    'pose_queue', self.on_pose_msgs)
        self.register_message_to_queue("MORSE_SET_Z",
                    'pose_queue', self.on_pose_msgs)
        self.register_message_to_queue("MORSE_SET_ROLL",
                    'pose_queue', self.on_pose_msgs)
        self.register_message_to_queue("MORSE_SET_PITCH",
                    'pose_queue', self.on_pose_msgs)
        self.register_message_to_queue("MORSE_SET_YAW",
                    'pose_queue', self.on_pose_msgs)

    def on_pose_msgs(self, msg):
        # look for position msgs
        if  (msg.key() == "MORSE_SET_X") and (msg.is_double()):
            self.data['x'] = msg.double() # robot X position [m]
        elif  (msg.key() == "MORSE_SET_Y") and (msg.is_double()):
            self.data['y'] = msg.double() # robot Y position [m]
        elif  (msg.key() == "MORSE_SET_Z") and (msg.is_double()):
            self.data['z'] = msg.double() # robot Z position [m]
        elif  (msg.key() == "MORSE_SET_ROLL") and (msg.is_double()):
            self.data['roll'] = msg.double() # robot roll [rad]
        elif  (msg.key() == "MORSE_SET_PITCH") and (msg.is_double()):
            self.data['pitch'] = msg.double() # robot pitch [rad]
        elif  (msg.key() == "MORSE_SET_YAW") and (msg.is_double()):
            self.data['yaw'] = msg.double() # robot yaw [rad]
