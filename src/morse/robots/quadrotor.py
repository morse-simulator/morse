import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.robot


class Quadrotor(morse.core.robot.Robot):
    """
    Definition of a generic **quadrotor** without ``Rigid Body`` physics.
    It is expected to be used with actuators such as:
     - :doc:`../actuators/stabilized_quadrotor`

    .. note::

        The rotation of the rotors is fixed and only for show. Its speed
        can be adjusted in the Logic panel in Blender (shown with
        :kbd:`F4`) when the **cube** object are selected.
    """

    _name = 'Quadrotor'

    def __init__(self, obj, parent=None):
        # Call the constructor of the parent class
        logger.info('%s initialization' % obj.name)
        super(self.__class__, self).__init__(obj, parent)

        logger.info('Component initialized')

    def default_action(self):
        """ Main function of this component. """
        pass
