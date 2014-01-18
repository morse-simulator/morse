import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.robot

class RMax(morse.core.robot.Robot):
    """
    Simple definition of the **Yamaha RMAX** unmanned helicopter.

    .. note::

        The rotation of the rotor is fixed and only for show. Its speed
        can be adjusted in the Logic panel in Blender (shown with
        :kbd:`F4`) when the rotor object is selected.
    """

    _name = 'Yamaha RMAX platform'

    def __init__(self, obj, parent=None):
        # Call the constructor of the parent class
        logger.info('%s initialization' % obj.name)
        morse.core.robot.Robot.__init__(self, obj, parent)

        logger.info('Component initialized')

    def default_action(self):
        """ Main function of this component. """
        pass
