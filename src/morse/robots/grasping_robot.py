import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.robot import Robot
from morse.core.services import service
from morse.core import blenderapi

class GraspingRobot(Robot):
    """ Class definition for a "virtual" robot.

    This robot class does not have a graphical representation,
    and it can not move.
    Its only purpose is to define the service grasp that may be
    used by other robot to pick up object.

    The robot need to have a "hand_name" with a near sensor in order to
    be able to use the grasp service.
    """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        Optionally it gets the name of the object's parent,
        but that information is not currently used for a robot.
        """
        # Call the constructor of the parent class
        logger.info('%s initialization' % obj.name)
        Robot.__init__(self, obj, parent)

        self.hand_name = 'todefine'

        self.bge_object['DraggedObject'] = None

        logger.info('Component initialized')

    @service
    def grasp(self, grab, obj_name=None):
        """
        Grasp near object.

        :param grab: set to True to take an object and False to release it.
        :param obj_name: when None, the robot will just grasp the nearest
                         object without any consideration for the object name.
        """
        logger.debug("morse grasp request received")
        grasping_robot = self.bge_object
        scene = blenderapi.scene()
        if self.hand_name == "todefine":
            logger.error("grasp failed because hand_name was not defined")
        hand_empty = scene.objects[self.hand_name]

        near_sensor = hand_empty.sensors['Near']

        if grab:
            logger.debug("seq t")
            # Check that no other object is being carried
            if (grasping_robot['DraggedObject'] is None or
            grasping_robot['DraggedObject'] == '') :
                logger.debug("Hand is free, I can grab")
                # If name was specified
                if obj_name:
                    near_objects = [obj for obj in near_sensor.hitObjectList
                                    if obj.name == obj_name]
                    if not near_objects:
                        logger.warning("no object named %s in %s"%(obj_name,
                              str(near_sensor.hitObjectList) ))
                        near_object = None
                    else:
                        near_object = near_objects[0]
                else:
                    near_object = near_sensor.hitObject

                hand_empty['Near_Object'] = near_object
                selected_object = hand_empty['Near_Object']
                # If the object is draggable
                if selected_object is not None and selected_object != '':
                    # Clear the previously selected object, if any
                    logger.debug("Object to grab is %s" % selected_object.name)
                    grasping_robot['DraggedObject'] = selected_object
                    # Remove Physic simulation
                    selected_object.suspendDynamics()
                    #Put object in the hand
                    selected_object.worldPosition = hand_empty.worldPosition
                    # Parent the selected object to the hand target
                    selected_object.setParent (hand_empty)
                    logger.debug( "OBJECT %s PARENTED TO %s" %
                                (selected_object.name, hand_empty.name) )
        else:
            if (grasping_robot['DraggedObject'] is not None and
            grasping_robot['DraggedObject'] != '') :
                previous_object = grasping_robot["DraggedObject"]
                # Restore Physics simulation
                previous_object.restoreDynamics()
                previous_object.setLinearVelocity([0, 0, 0])
                previous_object.setAngularVelocity([0, 0, 0])
                # Remove the parent
                previous_object.removeParent()
                # Clear the object from dragged status
                grasping_robot['DraggedObject'] = None
                logger.debug("JUST DROPPED OBJECT %s" % previous_object.name)

    def default_action(self):
        """ Main function of this component """
        pass
