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
# To use this script:
#   - create an 'empty' with the following sensors:
#     - a 'Left button' mouse sensor called 'rmb'
#     - a 'Over any' mouse sensor called 'overAny' in TRUE level triggering mode
#   These 2 sensors must be conencted to a Pythno actuator pointing to the 
#   "mouse_drag.dragDrop" module.
#
#   - object you want to drag must have a property named "draggable"
#
######################################################

import GameLogic

from Mathutils import Vector
import Rasterizer as render

motionSpeed = 5

#---- Just to make the cursor visible
def showMouse(cont):
    render.showMouse(1)
    print "show Mouse"
    
def hideMouse(cont):
    render.showMouse(0)
    print "hide Mouse"

#---- dragging functions        
def storeDraggedObject(own, hitObject):
    if hitObject != None:
        hitObject.suspendDynamics()
        own["draggedObject"] = hitObject
        return hitObject
    else: 
        formerObj = own["draggedObject"]
        formerObj.restoreDynamics()
        own["draggedObject"] = None
        return formerObj

def getTargetPosition(overAny, restriction, axis):
    
    # raySource = overAny.raySource #Blender 2.5
    raySource = Vector(overAny.raySource)
    
    # hitPosition = overAny.hitPosition #Blender 2.5
    hitPosition = Vector(overAny.hitPosition)
    rayVector = hitPosition-raySource
    
    if hitPosition[axis] > restriction:
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

    restrictedAxis = cont.owner["restrictedAxis"]
    restriction = draggedObject.worldPosition[restrictedAxis]
    targetPosition = getTargetPosition(overAny, restriction, restrictedAxis)
    
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
    if not "draggedObject" in cont.owner:
            cont.owner["draggedObject"] = None
    if not "restrictedAxis" in cont.owner:
            cont.owner["restrictedAxis"] = 2 #by default, restrict movemenent on (x,y)
    
    if activateDrag.triggered: #just activated
        if not cont.owner["dragging"] and activateDrag.positive and overAny.positive and "draggable" in overAny.hitObject:
            cont.owner["dragging"] = True
            print "Dragging on", overAny.hitObject
            storeDraggedObject(cont.owner, overAny.hitObject)
    
        if cont.owner["dragging"] and not activateDrag.positive: #desactivating
            cont.owner["dragging"] = False
            print "End dragging"
            storeDraggedObject(cont.owner, None)
            return
    
    if cont.owner["dragging"]: #activate
        doDrag(cont)
