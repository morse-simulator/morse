# Moos middleware datahandler for the Optix accelerated morse camera sensor

import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.moos import MOOSSubscriber, MOOSNotifier
from morse.core import blenderapi
import json

class optixCameraReader(MOOSSubscriber):
    """ Read optix camera commands and update local data. """

    def initialize(self):

        # initialize the parent class
        MOOSSubscriber.initialize(self)
        
        # register for control variables from the database
        self.register_message_to_queue('POPTIXCAMERASIM_STATUS','optic_camera_status_queue', self.on_optix_camera_msgs)

    def on_optix_camera_msgs(self, msg):
        if (msg.key() == 'POPTIXCAMERASIM_STATUS') and (msg.is_string()):
            self.data['status'] = msg.string()

        self._new_messages = True

        return True

class optixCameraNotifier(MOOSNotifier):
    """ Notify optix camera """

    def default(self, ci = 'unused'):

        # Don't send any message
        # if there's no optix camera pose data
        str = self.data['optix_camera_pose']

        ## ------
        #logger.debug('optixCameraNotifier is publishing!')
        #ts = self.data['timestamp']
        #
        ## optix camera message
        #msg_name = self.data['optix_camera_name'] + '_CAPTURE'
        #self.notify(msg_name, str ,ts)
        ## -----

        if str:

            #logger.debug('optixCameraNotifier is publishing!')
            ts = self.data['timestamp']

            # optix camera message
            msg_name = self.data['optix_camera_name'] + '_CAPTURE'
            self.notify(msg_name, str ,ts)

            # optix camera view message
            self.notify('OPTIX_CAMERA_VIEW', self.data['optix_camera_view'] ,ts)

            # Cancel message for next cycle
            self.data['optix_camera_pose'] = ''

    def update_morse_data(self):
        logger.debug('optixCameraNotifier.update_morse_data() called.')
