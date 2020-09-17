import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.moos import MOOSSubscriber
from morse.core import blenderapi
from math import degrees, radians

class FixedCtrlReader(MOOSSubscriber):
    """ Read thruster commands and update local data. """
 
    def initialize(self):

        # initialize the parent class
        MOOSSubscriber.initialize(self)
        
        # register for control variables from the database
        self.register_message_to_queue("DESIRED_RUDDER",'rudder_queue', self.on_thruster_msgs)
        self.register_message_to_queue("DESIRED_ELEVATOR",'elevator_queue', self.on_thruster_msgs)
        self.register_message_to_queue("DESIRED_THRUST",'thrust_queue', self.on_thruster_msgs)

    def on_thruster_msgs(self, msg):
        if (msg.key() == "DESIRED_RUDDER") and (msg.is_double()):
            #print('desired_rudder = ', msg.double() )
            self.data['desired_rudder'] = msg.double()
        elif  (msg.key() == "DESIRED_ELEVATOR") and (msg.is_double()):
            #print('desired_elevator = ', -radians(msg.double()) )
            self.data['desired_elevator'] = -radians(msg.double())
        elif  (msg.key() == "DESIRED_THRUST") and (msg.is_double()):
            #print('DESIRED_THRUST = ', msg.double() )
            self.data['desired_thrust'] = msg.double()

        self._new_messages = True
        return True

    def update_morse_data(self):
        logger.debug('ThrusterReader.update_morse_data() called.')

class TailconeCtrlReader(MOOSSubscriber):
    """ Read thruster commands and update local data. """
 
    def initialize(self):

        # initialize the parent class
        MOOSSubscriber.initialize(self)
        
        # register for control variables from the database
        self.register_message_to_queue("DESIRED_RUDDER",'rudder_queue', self.on_thruster_msgs)
        self.register_message_to_queue("DESIRED_ELEVATOR",'elevator_queue', self.on_thruster_msgs)
        self.register_message_to_queue("DESIRED_THRUST",'thrust_queue', self.on_thruster_msgs)

    def on_thruster_msgs(self, msg):
        if (msg.key() == "DESIRED_RUDDER") and (msg.is_double()):
            #print('desired_rudder = ', msg.double() )
            self.data['desired_rudder'] = radians(msg.double())
        elif  (msg.key() == "DESIRED_ELEVATOR") and (msg.is_double()):
            #print('desired_elevator = ', -radians(msg.double()) )
            self.data['desired_elevator'] = -radians(msg.double())
        elif  (msg.key() == "DESIRED_THRUST") and (msg.is_double()):
            #print('DESIRED_THRUST = ', msg.double() )
            self.data['desired_thrust'] = msg.double()

        self._new_messages = True
        return True

    def update_morse_data(self):
        logger.debug('ThrusterReader.update_morse_data() called.')