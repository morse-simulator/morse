import logging; logger = logging.getLogger("morse." + __name__)

######################################################
#
#    gripper.py        Blender 2.59
#
#    Gilberto Echeverria
#    13 / 10 / 2010
#
######################################################

import bge
import math
import morse.core.actuator
import morse.helpers.math as morse_math
from morse.core.services import service

class GripperActuatorClass(morse.core.actuator.MorseActuatorClass):
    """
    Basic gripper with two moving pieces

    It is capable of picking up objects marked with a predefined Game Property,
    currently they should be 'Graspable'
    """

    def __init__(self, obj, parent=None):
        """
        Constructor method.
        Receives the reference to the Blender object.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self.local_data['grab'] = False

        self._near_object = None
        self._grabbed_object = None

        logger.info('Component initialized')
        logger.setLevel(logging.DEBUG)


    def find_object(self):
        """ Store the object that is within reach of the gripper
        Uses a Blender Radar Sensor to detect objects with the
        'Graspable' property in front of this component
        """
        # Get reference to the Radar Blender sensor
        contr = bge.logic.getCurrentController()
        radar = contr.sensors['Radar']

        self._near_object = None
        if radar.triggered and radar.positive:
            min_distance = 100
            for test_obj in radar.hitObjectList:
                # Find the closest object and its distance
                new_distance = self.blender_obj.getDistanceTo(test_obj)
                if new_distance < min_distance:
                    self._near_object = test_obj
                    min_distance = new_distance


    @service
    def grab(self):
        """ Mark an object as selected by the user """
        # Check that no other object is being carried
        if self._grabbed_object == None:
            # If the object is draggable
            if self._near_object != None:
                logger.debug("Grabbing object: '%s'" % self._near_object)
                # Remove Physic simulation
                #self._near_object.suspendDynamics()
                # Parent the selected object to the gripper
                self._grabbed_object = self._near_object
                self._grabbed_object.setParent (self.blender_obj)
                logger.debug("Object: %s" % type(self._grabbed_object))
                logger.debug("New parent: %s" % self._grabbed_object.parent)
                self._near_object = None
                return True

        # Default response
        return False


    @service
    def release(self):
        """ Free the grabbed object
        Lets it fall down after reseting its rotation
        """
        # Clear the previously selected object, if any
        if self._grabbed_object != None:
            logger.debug("Releasing object: '%s'" % self._near_object)
            # Remove the parent
            self._grabbed_object.removeParent()
            # Place the object on the nearest surface
            #morse.helpers.place_object.do_place(previous_object)
            # Reset rotation of object
            self._grabbed_object.worldOrientation = [0.0, 0.0, 0.0]
            # Restore Physics simulation
            #previous_object.restoreDynamics()
            #previous_object.setLinearVelocity([0, 0, 0])
            #previous_object.setAngularVelocity([0, 0, 0])
            # Clear the object from dragged status
            self._grabbed_object = None
            return True

        # Default response
        return False


    def default_action(self):
        """
        Check if an object is within reach of the hand
        """
        self.find_object()
