import logging; logger = logging.getLogger("morse." + __name__)
import bge
import morse.core.robot
from morse.core.services import service

logger.setLevel(logging.DEBUG)

class MocapHumanClass(morse.core.robot.MorseRobotClass):
    """ Class definition for the human avatar as a robot entity
    Sub class of Morse_Object.
    This human is set to be controlled using data from an Xtion/Kinect sensor
    """

    def __init__(self, obj, parent=None):
        """ Constructor method.
            Receives the reference to the Blender object.
            Optionally it gets the name of the object's parent,
            but that information is not currently used for a robot. """
        # Call the constructor of the parent class
        logger.info('%s initialization' % obj.name)
        super(self.__class__,self).__init__(obj, parent)

        logger.info('Component initialized')

    @service
    def move(self, speed, rotation):
        """ Move the human.
        """
        human = self.blender_obj

        if not human['Manipulate']:
            human.applyMovement( [speed,0,0], True )
            human.applyRotation( [0,0,rotation], True )
        else :
            scene = bge.logic.getCurrentScene()
            target = scene.objects['IK_Target_Empty.R']

            target.applyMovement([0.0, rotation, 0.0], True)
            target.applyMovement([0.0, 0.0, -speed], True)


    @service
    def move_head(self, pan, tilt):
        """ Move the human head.
        """
        human = self.blender_obj
        scene = bge.logic.getCurrentScene()
        target = scene.objects['IK_Target_Empty.Head']

        if human['Manipulate']:
            return

        target.applyMovement([0.0, pan, 0.0], True)
        target.applyMovement([0.0, 0.0, tilt], True)

    @service
    def grasp_(self, seq):
        """ Grasp object.
        """
        scene = bge.logic.getCurrentScene()
        hand_empty = scene.objects['Hand_Grab.R']
        selected_object = hand_empty['Near_Object']
        human = self.blender_obj

        #if human['Manipulate']:
        if seq == "t":
            # Check that no other object is being carried
            if (human['DraggedObject'] == None or
            human['DraggedObject'] == '') :
                logger.debug("Hand is free, I can grab")
                # If the object is draggable
                if selected_object != None and selected_object != '':
                    # Clear the previously selected object, if any
                    logger.debug("Object to grab is %s" % selected_object.name)
                    human['DraggedObject'] = selected_object
                    # Remove Physic simulation
                    selected_object.suspendDynamics()
                    # Parent the selected object to the hand target
                    selected_object.setParent (hand_empty)
                    logger.debug ("OBJECT %s PARENTED TO %s" % (selected_object.name, hand_empty.name))

        if seq == "f":
            if (human['DraggedObject'] != None and
            human['DraggedObject'] != '') :
                previous_object = human["DraggedObject"]
                # Restore Physics simulation
                previous_object.restoreDynamics()
                previous_object.setLinearVelocity([0, 0, 0])
                previous_object.setAngularVelocity([0, 0, 0])
                # Remove the parent
                previous_object.removeParent()
                # Clear the object from dragged status
                human['DraggedObject'] = None
                logger.debug ("JUST DROPPED OBJECT %s" % (previous_object.name))

    @service
    def move_hand(self, diff, tilt):
        """ move the human hand (wheel). a request to use by a socket.
        Done for wiimote remote control.
        """
        human = self.blender_obj
        if human['Manipulate']:
            scene = bge.logic.getCurrentScene()
            target = scene.objects['IK_Target_Empty.R']
            target.applyMovement([diff, 0.0, 0.0], True)

    @service
    def toggle_manipulation(self):
        """ Switch manipulation mode on and off. a request to use by a socket.
        Done for wiimote remote control.
        """
        human = self.blender_obj
        scene = bge.logic.getCurrentScene()
        hand_target = scene.objects['IK_Target_Empty.R']
        head_target = scene.objects['IK_Target_Empty.Head']
        torso = scene.objects['Torso_Reference_Empty']

        if human['Manipulate']:
            human['Manipulate'] = False
            # Place the hand beside the body
            hand_target.localPosition = [0.3, -0.3, 0.9]
            # Make the head follow the body
            head_target.setParent(torso)
            # Put the head_target in front and above the waist
            head_target.localPosition = [0.5, 0.0, 0.5]
            logger.debug("Moving head_target to CENTER: %s" % head_target.localPosition)

            #hand_target.localPosition = [0.0, -0.3, 0.8]
            #head_target.setParent(human)
            #head_target.localPosition = [1.3, 0.0, 1.7]
        else:
            human['Manipulate'] = True
            # Place the hand in a nice position
            hand_target.localPosition = [0.6, 0.0, 1.4]
            # Make the head follow the hand
            head_target.setParent(hand_target)
            # Reset the head_target position to the same as its parent
            head_target.localPosition = [0.0, 0.0, 0.0]
            logger.debug("Moving head_target to HAND: %s" % head_target.localPosition)


            #head_target.setParent(hand_target)
            # Place the hand in a nice position
            #hand_target.localPosition = [0.6, 0.0, 1.4]
            # Place the head in the same place
            #head_target.localPosition = [0.0, 0.0, 0.0]

    @service
    def switch_cameras(self):
        """ Change between the main camera view in MORSE and the first person camera """
        scene = bge.logic.getCurrentScene()
        index = bge.logic.current_camera_index
        next_camera = scene.cameras[index]
        scene.active_camera = next_camera
        logger.info("Showing view from camera: '%s'" % next_camera.name)
        # Disable mouse cursor for Human camera
        if next_camera.name == "Human_Camera":
            bge.logic.mouse.visible = False
        else:
            bge.logic.mouse.visible = True
        # Update the index for the next call
        index = (index + 1) % len(scene.cameras)
        bge.logic.current_camera_index = index



    def default_action(self):
        """ Main function of this component. """
        pass
