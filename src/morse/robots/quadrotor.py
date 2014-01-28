import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.robot


class Quadrotor(morse.core.robot.Robot):
    """
    Definition of a generic **quadrotor** without ``Rigid Body`` physics.
    It is expected to be used with actuators such as:

    - :doc:`../actuators/stabilized_quadrotor`

    .. example::
        from morse.builder import *

        quadrotor = QUAD2012()

        # place your component at the correct location
        quadrotor.translate(<x>, <y>, <z>)
        quadrotor.rotate(<rx>, <ry>, <rz>)

        # define one or several communication interface, like 'socket'
        quadrotor.add_interface(<interface>)

        env = Environment('empty')


    .. note::

        The rotation of the rotors is fixed and only for show. Its speed
        can be adjusted in the Logic panel in Blender (shown with
        :kbd:`F4`) when the **cube** object are selected.

    :noautoexample:
    """

    _name = 'Quadrotor'

    def __init__(self, obj, parent=None):
        # Call the constructor of the parent class
        logger.info('%s initialization' % obj.name)
        morse.core.robot.Robot.__init__(self, obj, parent)

        logger.info('Component initialized')

    def default_action(self):
        """ Main function of this component. """
        pass
