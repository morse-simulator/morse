import logging; logger = logging.getLogger("morse." + __name__)
import datetime

import morse.core.middleware

from morse.middleware.pocolibs.sensors.General_Poster import ors_poster


class MorsePocolibsClass(morse.core.middleware.MorseMiddlewareClass):
    """ Handle communication between Blender and Pocolibs."""

    def __init__(self):
        # Call the constructor of the parent class
        super(self.__class__,self).__init__()

        # Store the id's of created posters, indexed by component name
        self._poster_dict = dict()
        # Dictionary for external posters
        self._poster_in_dict = dict()
        self._imported_modules = dict()


    def __del__(self):
        """ Close all open posters. """
        for component_name, poster_id in self._poster_dict.items():
            logger.info("Killing poster %d for component %s" % (poster_id, component_name))
            # Call the method to close a poster
            ors_poster.finalize(poster_id)


    def register_component(self, component_name, component_instance, mw_data):
        """ Open the pocolibs poster to communicate a specified component.

        The configuration of each type of poster is done in external modules.
        """
        # Import the configuration of the poster from an extra module
        function = self._add_method(mw_data, component_instance)


    def _compute_date(self):
        """ Compute the current time

        ( we only require that the date
        increases using a constant step so real time is ok)
        """
        t = datetime.datetime.now()
        date = int(t.hour * 3600* 1000 + t.minute * 60 * 1000 +
                t.second * 1000 + t.microsecond / 1000)

        return date, t


def init_extra_actuator(self, component_instance, function, mw_data, kind):
    """ Setup the middleware connection with this data
    Prepare the middleware to handle the serialised data as necessary.
    """
    component_name = component_instance.blender_obj.name
    parent_name = component_instance.robot_parent.blender_obj.name
    # Check if the name of the poster has been given in mw_data
    try:
        # It should be the 4th parameter
        poster_name = mw_data[3]
    except IndexError as detail:
        # Compose the name of the poster, based on the parent and module names
        poster_name = '{0}_{1}'.format(parent_name, component_name)

    logger.debug("Creating poster_name %s" % poster_name)
    poster_id = kind.createPosterHandler(poster_name)
    self._poster_in_dict[component_name] = poster_id
    component_instance.input_functions.append(function)
