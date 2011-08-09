import logging; logger = logging.getLogger("morse." + __name__)
######################################################
#
#    mouse_drag.py        Blender 2.49
#
#    A script to pick with the mouse and move in 3D
#    an object
#
# Based on previous work by Monster, Almost and Stralen
# http://blenderartists.org/forum/showthread.php?p=1688086
#
#    Severin Lemaignan
#    24 / 08 / 2010
#
######################################################

import GameLogic
import Rasterizer
from mathutils import Vector

motionSpeed = 5
rotationSpeed = 5
#find point on 2D plane of fixed z
#that is between the camera and the hitposition
lowestLevel = 0.5 #this is the fixed z  value



def init(contr):
    #If necessary, initialize the property
    if not "dragging" in contr.owner:
        contr.owner["dragging"] = False
    if not "restrictedAxis" in contr.owner:
        #by default, restrict movemenent on (x,y)
        contr.owner["restrictedAxis"] = 1
    if not "draggedObject" in contr.owner:
        contr.owner["draggedObject"] = None
    if not "motionPlane" in contr.owner:
        contr.owner["motionPlane"] = None

    showMouse(contr)


#---- debugging function not really needed
def moveToTargetPosition(cont):
    '''
    Moves the owner of the controller to the position stored
    in the property "targetPosition" of the owner of the first sensor.
    '''
    try:
        cont.owner.worldPosition = cont.sensors[0].owner["targetPosition"]
    except KeyError:
        pass


#---- utility function
def onePositive(contr):
    for sensor in contr.sensors:
        if sensor.positive:
            return True
    return False


#---- Just to make the cursor visible
def showMouse(contr):
    if not onePositive(contr):
        return
    Rasterizer.showMouse(1)
    logger.info("Showing Mouse")


def hideMouse(cont):
    if not onePositive(cont):
        return
    Rasterizer.showMouse(0)
    logger.info("hide Mouse")


def getTargetPosition(overAny, basePosition, motionPlane):
    """ Compute the new location for the selected object """
    # Go back to basics. Simply place the object where the mouse is
    if overAny.hitObject == motionPlane:
        targetPosition = Vector(overAny.hitPosition)
    else:
        targetPosition = basePosition

    return targetPosition


def doDrag(cont):
    overAny = cont.sensors["overAny"]

    draggedObject = cont.owner.get("draggedObject", None)
    if draggedObject == None:
        #Nothing to drag
        return

    logger.debug"Dragging", draggedObject, "on", overAny.hitObject

    if overAny.positive:
        restrictedAxis = cont.owner["restrictedAxis"]
        basePosition = draggedObject.position
        targetPosition = getTargetPosition(overAny, basePosition, cont.owner['motionPlane'])

        #just for debugging see moveToTargetPosition
        cont.owner["targetPosition"] = targetPosition
    else:
        return

    draggedObject.position = targetPosition


#---- All in one
def dragDrop(contr):
    """ Drag of the selected object """
    overAny = contr.sensors["overAny"]
    activateDrag = contr.sensors["LMB"]

    scene = GameLogic.getCurrentScene()
    XYPlane = scene.objects['XYPlane']
    XZPlane = scene.objects['XZPlane']
    YZPlane = scene.objects['YZPlane']

    # LMB was pressed
    if activateDrag.triggered and activateDrag.positive:
        logger.info("Dragging on", overAny.hitObject)
        contr.owner["dragging"] = True

        # Select the plane to be used for motion
        if contr.owner['restrictedAxis'] == 0:
            contr.owner['motionPlane'] = XYPlane
            logger.info("Using plane XY")
        if contr.owner['restrictedAxis'] == 1:
            contr.owner['motionPlane'] = XZPlane
            logger.info("Using plane XZ")
        if contr.owner['restrictedAxis'] == 2:
            contr.owner['motionPlane'] = YZPlane
            logger.info("Using plane YZ")

        # Move the motion plane to the location of the target
        contr.owner['motionPlane'].position = contr.owner['draggedObject'].position

    # LMB was released
    if activateDrag.triggered and not activateDrag.positive:
        logger.info("End dragging")
        contr.owner["dragging"] = False
        #storeDraggedObject(cont.owner, None)
        return

    # Move the object around
    if contr.owner["dragging"]:
        doDrag(contr)


def objectSelect(contr):
    """ Mark an object as selected by the user """
    scene = GameLogic.getCurrentScene()
    sphere = scene.objects['SelectionSphere']

    rightButton = contr.sensors["RMB"]
    # RMB was pressed
    if rightButton.triggered and rightButton.positive:
        overAny = contr.sensors["overAny"]
        selectedObject = overAny.hitObject

        # Clear the previously selected object, if any
        if contr.owner['draggedObject'] != None:
            previousObject = contr.owner["draggedObject"]
            # Restore Physics simulation
            previousObject.restoreDynamics()
            previousObject.setLinearVelocity([0, 0, 0])
            previousObject.setAngularVelocity([0, 0, 0])
            # Reset the sphere
            sphere.removeParent()
            sphere.position = [0, 0, 20]

        # Hide the motion plane previously used
        if contr.owner['motionPlane'] != None:
            contr.owner['motionPlane'].position = [0, 0, -20]

        # If the object is draggable
        if "draggable" in selectedObject:
            # Store the object selected
            contr.owner["draggedObject"] = selectedObject
            # Remove Physic simulation
            selectedObject.suspendDynamics()
            logger.info("SELECTED OBJECT: %s" % selectedObject)

            # Move the sphere to the location of the target
            sphere.position = selectedObject.position
            # Parent the sphere to selected object
            sphere.setParent (selectedObject)


def togglePlane(contr):
    """ Change the motion plane """
    planeIndex = contr.owner['restrictedAxis']
    planeIndex = (planeIndex + 1) % 3
    contr.owner['restrictedAxis'] = planeIndex
