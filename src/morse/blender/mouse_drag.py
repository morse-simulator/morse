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
from Mathutils import Vector

motionSpeed = 5
rotationSpeed = 5
#find point on 2D plane of fixed z
#that is between the camera and the hitposition
lowestLevel = 0.5 #this is the fixed z  value

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
def onePositive(cont):
    for sensor in cont.sensors:
        if sensor.positive:
            return True
    return False
#---- Just to make the cursor visible
def showMouse(cont):
    if not onePositive(cont):
        return
    Rasterizer.showMouse(1)
    print "show Mouse"
    
def hideMouse(cont):
    if not onePositive(cont):
        return
    Rasterizer.showMouse(0)
    print "hide Mouse"

#---- dragging functions        
def storeDraggedObject(own, hitObject):
    if hitObject != None and "draggable" in hitObject:
        own["draggedObject"] = hitObject
        return hitObject
    else: 
        formerObj = own["draggedObject"]
        own["draggedObject"] = None
        return formerObj

def getTargetPosition(overAny, restriction, axis):
    
    raySource = Vector(overAny.raySource)
    hitPosition = Vector(overAny.hitPosition)
    rayVector = hitPosition-raySource
    
    if not "drag" in overAny.hitObject \
    and hitPosition[axis] > restriction:
        targetPosition = hitPosition
    else:
        planeContactPoint = ( restriction-raySource[axis])/rayVector[axis]
        targetPosition = (rayVector*planeContactPoint) + raySource
        
    return targetPosition

def doDrag(cont):
    overAny = cont.sensors["overAny"]
    
    draggedObject = cont.owner.get("draggedObject", None)
    if draggedObject == None:
        #Nothing to drag
        return


    #print "Dragging", draggedObject, "on", overAny.hitObject   

    if overAny.positive:
        restrictedAxis = cont.owner["restrictedAxis"]
        restriction = draggedObject.worldPosition[restrictedAxis]
        targetPosition = getTargetPosition(overAny, restriction, restrictedAxis)

        #just for debugging see moveToTargetPosition
        cont.owner["targetPosition"] = targetPosition
    else:
        return
    
    draggedObject.position = targetPosition
    #currentPosition = Vector(draggedObject.position)
    #rayToWall = draggedObject.rayCast(targetPosition,draggedObject,0,'wall')
    
    #if rayToWall[0] == None:
    #    targetPosition = targetPosition
    #else:
    #    targetPosition = rayToWall[1]
        
    #speed = motionSpeed
    
    #vec = (Vector(targetPosition) - currentPosition)*speed
    
    ## only move in motion mode
    #if not activateRotation.positive:
    #    draggedObject.setLinearVelocity(vec,0)

#---- All in one    
def dragDrop(cont):
    overAny = cont.sensors["overAny"]
    activateDrag = cont.sensors["rmb"] 
    
    #If necessary, initialize the property
    if not "dragging" in cont.owner:
            cont.owner["dragging"] = False
    if not "restrictedAxis" in cont.owner:
            cont.owner["restrictedAxis"] = 2 #by default, restrict movemenent on (x,y)
    
    if activateDrag.triggered and activateDrag.positive: #just activated        
        cont.owner["dragging"] = not cont.owner["dragging"]
        
        #initialize dragging
        if cont.owner["dragging"]:
            print "Dragging on", overAny.hitObject
            storeDraggedObject(cont.owner, overAny.hitObject)
            
        else:
            print "End dragging"
            storeDraggedObject(cont.owner, None)
            return
    
    
    if cont.owner["dragging"]: #activate
        doDrag(cont)
