import logging; logger = logging.getLogger("morse." + __name__)

######################################################
#
#    gripper.py        Blender 2.59
#
#    Gilberto Echeverria
#    13 / 10 / 2010
#
######################################################

import morse.core.actuator
from morse.core import blenderapi
from morse.core.services import service
from morse.helpers.components import add_data, add_property

class Gripper(morse.core.actuator.Actuator):
    """
    Actuator capable of grabbing objects marked with the ``Graspable``
    Game Property.  Currently it only works using services: **grab** and
    **release**.  When instructed to grab an object, it will check if it
    is within range, and if so, will parent the grabbed object to
    itself.

    .. note::

        For objects to be detected and grabbed by the gripper, they must
        have the following settings in the **Physics Properties** panel:

            - **Actor** must be checked
            - **Collision Bounds** must be checked
            - **Physics Type** must be ``Rigid Body``

        This will work even for Static objects

    .. warning::

        This actuator does not simulate the physical interaction of the
        gripper fingers with the objects it grabs. Its purpose is to
        abstract the action of taking an object, for human-robot
        interaction experiments.
    """

    _name = "Gripper"
    _short_desc = "Instruct the robot to move towards a given target"

    add_data('grab', False, "bool", "Currently not used")

    # These properties are not used directly in the logic, but are used
    # in the builder to create the radar properly.
    # These value cannot be changed dynamically in bge.
    add_property('_angle', 60.0, 'Angle', 'float',
                 'Aperture angle of the radar capable to detecting the \
                  graspable objects (in degree)')
    add_property('_distance', 0.5, 'Distance', 'float',
                 'Detection distance in meter. Graspable objects further \
                 way from the gripper than this distance cannot be  \
                 held')

    def __init__(self, obj, parent=None):
        """
        Constructor method.
        Receives the reference to the Blender object.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        self._near_object = None
        self._grabbed_object = None

        # Variable to indicate whether the mesh should animate closing
        #  or opening the gripper
        self._animation = ''

        # Get references to the Logic Bricks to play animations
        self._contr = self.bge_object.controllers[0]
        self._close_anim = self._contr.actuators['Close_anim']
        self._open_anim = self._contr.actuators['Open_anim']

        logger.info('Component initialized')
        logger.setLevel(logging.DEBUG)


    def find_object(self):
        """
        Store the object that is within reach of the gripper Uses a
        Blender Radar Sensor to detect objects with the 'Graspable'
        property in front of this component
        """
        # Get reference to the Radar Blender sensor
        contr = blenderapi.controller()
        radar = contr.sensors['Radar']

        self._near_object = None
        if radar.triggered and radar.positive:
            min_distance = 100
            for test_obj in radar.hitObjectList:
                # Find the closest object and its distance
                new_distance = self.bge_object.getDistanceTo(test_obj)
                if new_distance < min_distance:
                    self._near_object = test_obj
                    min_distance = new_distance


    @service
    def grab(self):
        """
        Tries to grab an object close to the gripper.

        :returns: if successful (or if an object is already in hand), the
                  name of the object, else None.
        """
        # Check that no other object is being carried
        if not self._grabbed_object:
            # If the object is draggable
            if self._near_object != None:
                logger.debug("Grabbing object: '%s'" % self._near_object)
                # Remove Physic simulation
                #self._near_object.suspendDynamics()
                # Parent the selected object to the gripper
                self._grabbed_object = self._near_object
                self._grabbed_object.setParent (self.bge_object)
                logger.debug("New parent: %s" % self._grabbed_object.parent)
                self._near_object = None

                # Execute the close grip animation:
                self._animation = 'close'
                return self._grabbed_object.name

            else:
                logger.debug("No 'Graspable' object within range of gripper")
                return None
        else:
            logger.debug("Already holding object %s" % self._grabbed_object )
            return self._grabbed_object.name

    @service
    def release(self):
        """
        Free the grabbed object.

        Let it fall down after resetting its rotation.
        Does nothing if no object is held.

        :returns: True if an object has been released, else False (if
                  no object was held).
        """
        # Clear the previously selected object, if any
        if self._grabbed_object != None:
            logger.debug("Releasing object: '%s'" % self._near_object)
            # Remove the parent
            self._grabbed_object.removeParent()
            # Place the object on the nearest surface
            #morse.helpers.place_object.do_place(previous_object)
            # Reset rotation of object
            #self._grabbed_object.worldOrientation = [0.0, 0.0, 0.0]
            # Restore Physics simulation
            #previous_object.restoreDynamics()
            #previous_object.setLinearVelocity([0, 0, 0])
            #previous_object.setAngularVelocity([0, 0, 0])
            # Clear the object from dragged status
            self._grabbed_object = None

            # Execute the open grip animation:
            self._animation = 'open'
            return True

        else:
            logger.debug("No object currently being held: nothing to release.")
            return False

    def default_action(self):
        """
        Check if an object is within reach of the hand
        """
        self.find_object()

        # Play the animations when necessary
        if self._animation == 'close':
            self._contr.activate(self._close_anim)
            self._animation = ''
            logger.debug('Playing CLOSE animation')
        if self._animation == 'open':
            self._contr.activate(self._open_anim)
            self._animation = ''
            logger.debug('Playing OPEN animation')
