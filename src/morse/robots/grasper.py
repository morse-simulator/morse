import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.robot import Robot
from morse.core.services import service
from morse.core import blenderapi

class RobotGrasper(Robot):
    """ Class definition for a "virtual" robot.

    This robot class does not have a graphical representation,
    and it can not move.
    Its only purpose is to define the service grasp that may be
    used by other robot to pick up object.

    Sub class of Morse_Object. """

    def __init__(self, obj, parent=None):
        """ Constructor method.
            Receives the reference to the Blender object.
            Optionally it gets the name of the object's parent,
            but that information is not currently used for a robot. """
        # Call the constructor of the parent class
        logger.info('%s initialization' % obj.name)
        Robot.__init__(self, obj, parent)

        self.hand_name = 'todefine'
        logger.info('Component initialized')

    @service
    def grasp_(self, seq):
        """ Grasp object
        """
        logger.debug("morse grasp request received")
        robot_grasper = self.bge_object
        scene = blenderapi.scene()
        if self.hand_name == "todefine":
            logger.error("grasp failed because hand_name was not defined")
        hand_empty = scene.objects[self.hand_name]

        near_sensor = hand_empty.sensors['Near']
        near_object = near_sensor.hitObject
        hand_empty['Near_Object'] = near_object

        selected_object = hand_empty['Near_Object']
        if seq == "t":
            logger.debug("seq t")
            # Check that no other object is being carried
            if (robot_grasper['DraggedObject'] == None or
            robot_grasper['DraggedObject'] == '') :
                logger.debug("Hand is free, I can grab")
                # If the object is draggable
                if selected_object != None and selected_object != '':
                    # Clear the previously selected object, if any
                    logger.debug("Object to grab is %s" % selected_object.name)
                    robot_grasper['DraggedObject'] = selected_object
                    # Remove Physic simulation
                    selected_object.suspendDynamics()
                    # Parent the selected object to the hand target
                    selected_object.setParent (hand_empty)
                    logger.debug ("OBJECT %s PARENTED TO %s" % (selected_object.name, hand_empty.name))

        if seq == "f":
            if (robot_grasper['DraggedObject'] != None and
            robot_grasper['DraggedObject'] != '') :
                previous_object = robot_grasper["DraggedObject"]
                # Restore Physics simulation
                previous_object.restoreDynamics()
                previous_object.setLinearVelocity([0, 0, 0])
                previous_object.setAngularVelocity([0, 0, 0])
                # Remove the parent
                previous_object.removeParent()
                # Clear the object from dragged status
                robot_grasper['DraggedObject'] = None
                logger.debug ("JUST DROPPED OBJECT %s" % (previous_object.name))


    def default_action(self):
        """
        Main function of this component
        """
        pass

