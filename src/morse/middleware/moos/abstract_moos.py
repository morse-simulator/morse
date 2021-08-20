import logging; logger = logging.getLogger("morse.moos")
try:
    from pymoos import pymoos
except:
    import pymoos
from morse.middleware import AbstractDatastream
from morse.core import blenderapi


class AbstractMOOS(AbstractDatastream):
    """ Base class for all MOOS Publishers and Subscribers """
    # used to generate documentation, TODO fill in subclasses
    _type_name = "MOOS Communications"
    _type_url = ""
    _moos_comms = {}
    _new_messages = False

    def initialize(self):
        """ Initialize the MOOS comms. """
        logger.info("MOOS datastream initialize %s"%self)

        self.moos_host = self.kwargs.get('moos_host', '127.0.0.1')
        self.moos_port = self.kwargs.get('moos_port', 9000)
        self.moos_name = self.kwargs.get('moos_name', 'iMorse')

        key = (self.moos_name)

        if not key in self._moos_comms:
            # initializing the MOOS Comms
            self._moos_comms[key] = pymoos.comms()
            # linking callback when connected to MOOSDB
            self._moos_comms[key].set_on_connect_callback(self.on_connect)
            # linking callback for new mail
            self._moos_comms[key].set_on_mail_callback(self.on_mail)
            # starting MOOS Comms
            self._moos_comms[key].run(self.moos_host,
                                            self.moos_port,
                                            self.moos_name)
            logger.info("MOOS datastream launched: name=%s"%(self.moos_name))

        # all instance share the same static MOOSApp according to host and port
        self._comms = self._moos_comms[key]

    def on_connect(self):
        """ On connect to server callback """
        logger.info(
            "MOOS datastream connected: name= %s host=%s:port=%d."%(
                self.moos_name,self.moos_host, self.moos_port))
        return True

    def on_mail(self):
        """ On new mail callback.

        Needs to be overloaded if not using callbacks.
        Warning: only one on_mail should be called per AbstractMOOS._comms[key]
        """
        # for msg in self._comms.fetch():
        #     logger.info("MOOS datastream name= %s got mail: %s."%(
        #                                 self.moos_name, msg.trace()))
        # return True
        self._new_messages = True
        pass

    def finalize(self):
        """ Kill the morse MOOS comms."""
        key = (self.moos_name)
        if key in self._moos_comms:
            self._moos_comms[key].close(True)
            self._moos_comms.pop(key)
            logger.info("MOOS datastream finalized: %s:%d"
                %(self.moos_host, self.moos_port))


class MOOSSubscriber(AbstractMOOS):
    """ Base class for all MOOS Subscribers """

    def register(self, msg_key, min_time=0.):
        """ Register to MOOS message.
        This is just a wrapper so _comms is not directly called

        :param msg_key: MOOS message to register.
        :param min_time: minimun time between notifications in second (0.0)
        """
        logger.info("MOOSSubscriber registering for %s"%(msg_key))
        while not self._comms.is_connected():
            pass
        return self._comms.register(msg_key, min_time)

    def register_message_to_queue(self, msg_key, queue_name, queue_callback,
                                    min_time=0.):
        if not self._comms.add_active_queue(queue_name, queue_callback):
            logger.warning('MOOSSubscriber.register_message_to_queue(): '
                            'queue with similar name exists. '
                            'Old queue has priority!')
        if not self._comms.add_message_route_to_active_queue(queue_name,
                                            msg_key):
            logger.error('MOOSSubscriber.register_message_to_queue(): '
                            'Couldn\'t add %s to %s queue'%(msg_key,
                                                            queue_name))
            raise error
        return self.register(msg_key, min_time)

    def update_morse_data(self):
        """ Update MORSE simulation with data from :param msg:

        Called when component.default_action is triggered
        and a new message was received
        """
        logger.debug('MOOSSubscriber.update_morse_data() activated.')
        pass

    def default(self, ci='unused'):
        # when new messages MORSE need update
        if self._new_messages:
            self.update_morse_data()
            self._new_messages = False
            return True
        return False


class MOOSNotifier(AbstractMOOS):
    """ Base class for all MOOS Publishers """

    def notify(self, msg_key, data, time=-1):
        """ Publishe new data as MOOSMsg.
        This is basically is a wrapper to avoid calling _comms

        :param msg_key: MOOSMsg key to notify
        :param data: value to be sent to MOOSDB (the type is defined by MOOSDB)
        :param time: time stamp of the message (-1 means now)
        """
        self._comms.notify(msg_key, data, time)

    def notify_binary(self, msg_key, data, time=-1):

        self._comms.notify_binary(msg_key, data, time)

class StringPublisher(MOOSNotifier):
    """ Publish a string containing a printable representation of the
    local data. """

    def default(self, ci='unused'):
        logger.debug("MOOSNotifier posting message to the MOOS DB.")
        #iterate through all objects of the component_instance and post the data
        for variable, data in self.data.items():
            name = "MORSE_%s_%s" % (self.component_name, variable)
            # MOOS Variables need to be in upper case per convention
            name = name.upper()
            # MOOS Variables should only use _
            name = name.replace('.','_')
            logger.debug("AbstractMOOS to publish:"
                        "name: %s, type: %s, data: %s"%(name,
                                                    type(data), str(data)))
            self.notify(name, str(data))


class StringReader(MOOSSubscriber):
    """ Log messages. """

    def update_morse_data(self):
        logger.debug("StringReader.update_morse_data() called." )
