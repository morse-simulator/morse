import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.pocolibs_mw import init_extra_actuator
from morse.middleware.pocolibs.actuators.Lwr_Poster import ors_lwr_poster

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    init_extra_actuator(self, component_instance, function, mw_data, ors_lwr_poster)

def read_lwr_config(self, component_instance):
    """ Read the angles for the segments of the Kuka arm """
    # Read from the poster specified
    poster_id = self._poster_in_dict[component_instance.blender_obj.name]
    gbm_conf, ok = ors_lwr_poster.read_lwr_data(poster_id)

    logger.debug("DATA READ by LWR: ", component_instance.local_data)

    if ok != 0:
        component_instance.local_data['seg0'] = gbm_conf.q1
        component_instance.local_data['seg1'] = gbm_conf.q2
        component_instance.local_data['seg2'] = gbm_conf.q3
        component_instance.local_data['seg3'] = gbm_conf.q4
        component_instance.local_data['seg4'] = gbm_conf.q5
        component_instance.local_data['seg5'] = gbm_conf.q6
        component_instance.local_data['seg6'] = gbm_conf.q7

        # Return true to indicate that a command has been received
        return True
    else:
        return False
