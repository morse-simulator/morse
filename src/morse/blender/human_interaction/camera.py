from bge import logic
from mathutils import Vector, Matrix

def init():
    """
    Sets the camera on load
    """
    co =  logic.getCurrentController()
    ow = co.owner
    camAct = co.actuators['Set_Camera']
    sobList =  logic.getCurrentScene().objects
    humCam = sobList['Human_Camera']
    
    try:
        worldCam = sobList['CameraFP']
        #check if there is a Camera displaying the world in the scene
    except KeyError:
        worldCam = None

    if ow['WorldCamera'] and worldCam:
        camAct.camera = worldCam
    else:
        camAct.camera = humCam
    # set Camera following the human or displaying the world
    
    
    co.activate(camAct)

def collision():
    """
    Detect camera collision and place the camera accordingly
    """
    co =  logic.getCurrentController()
    ow = co.owner
    ray = co.sensors['collision']
    right = co.sensors['RIGHT']
    left = co.sensors['LEFT']
    human =   logic.getCurrentScene().objects['Human']
    Orig_Pos = logic.getCurrentScene().objects['POS_3P_Camera_Orig']
    distance = 0.05     # the distance the camera keeps to Objects

    # if near an object, place the camera slightly in front of it
    if ray.positive and not human['Manipulate'] and not (right.positive or
                                                         left.positive):
        hitPos = ray.hitPosition
        ow.worldPosition = (Vector(hitPos) -
                            Vector(ray.rayDirection).normalized()*distance)
        ow['prop_collision'] = True
        
    elif ray.positive and not human['Manipulate'] and right.positive:
        hitPos = (Vector(ray.hitPosition) + Vector(right.hitPosition))/2
        ow.worldPosition = (hitPos -
                            (Vector(ray.rayDirection) +
                             Vector(right.rayDirection)).normalized()*distance)
        ow['prop_collision'] = True
        
    elif ray.positive and not human['Manipulate'] and left.positive:
        hitPos = (Vector(ray.hitPosition) + Vector(left.hitPosition))/2
        ow.worldPosition = (hitPos -
                            (Vector(ray.rayDirection) +
                             Vector(left.rayDirection)).normalized()*distance)
        ow['prop_collision'] = True
        
    elif left.positive and not human['Manipulate'] and not (right.positive or
                                                            ray.positive):
        hitPos = left.hitPosition
        ow.worldPosition = (Vector(hitPos) -
                            Vector(left.rayDirection).normalized()*distance)
        ow['prop_collision'] = True
        
    elif right.positive and not human['Manipulate'] and not (left.positive or
                                                             ray.positive):
        hitPos = right.hitPosition
        ow.worldPosition = (Vector(hitPos) -
                            Vector(right.rayDirection).normalized()*distance)
        ow['prop_collision'] = True

    else:
        ow['prop_collision'] = False
        ow.worldPosition = Orig_Pos.worldPosition
    # else place the camera to its former position 


def change():
    """
    Changes camera position to 1st person while in Manipulation-Mode
    """
    co = logic.getCurrentController()
    ow = co.owner
    track = co.actuators['TrackCamera']
    sobList = logic.getCurrentScene().objects
    human = sobList['Human']
    FP = sobList['POS_1P_Camera']
    FP_POS = sobList['POS_1P_Camera'].worldPosition
    TP_POS = sobList['POS_3P_Camera'].worldPosition
    head_target = sobList['Target_Empty']
    hand_target = sobList['IK_Target_Look.R']
    look_target = sobList['LOOK_TARGET']
    right_hand = sobList['Hand_Grab.R']
    mmb = ow.sensors['MMB']


    if ow.getDistanceTo(FP) < 0.08:
        # if the distance is smaller than 0.08,
        # snap the camera to the 'POS_1P_Camera'-empty
        ow['FP'] = True


    if not human['Manipulate']:
        ow['FP'] = False
        if not ow['prop_collision']:
            smooth_move(TP_POS, ow)
    else:
        if not ow['FP']:
            smooth_move(FP_POS, ow)
        else:
            ow.worldPosition = FP_POS


    # camera points to several empties according to the current situation
    if human['Manipulate']:
        track.object = head_target
    else:
        track.object = look_target
    co.activate(track)

def smooth_move(goal, owner):
    """This function moves this script's owner to 'goal'
    Change the smoothness to make the transition faster or slower
    higher value --> slower
    lower value --> faster
    recommended value: 10
    """
    smoothness = 10
    currLoc = owner.worldPosition
    destLoc = goal
    dX, dY, dZ = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    
    dX = currLoc[0] - destLoc[0]        
    dY = currLoc[1] - destLoc[1]        
    dZ = currLoc[2] - destLoc[2]
    smoothVec = currLoc
    
    smoothVec[0] = smoothVec[0] - dX/smoothness         
    smoothVec[1] = smoothVec[1] - dY/smoothness         
    smoothVec[2] = smoothVec[2] - dZ/smoothness
    owner.worldPosition = smoothVec


def raylength():
    """
    Objects can only be grabbed if they are hit by the Ray-Sensor called 'Ray'.
    Set the ray's length ,
    so that it hits objects in a certain radius around the human's z-axis
    """
    co = logic.getCurrentController()
    cam = co.owner
    ray = co.sensors['Ray']

    dir = Vector(ray.rayDirection)
    xy = Matrix.OrthoProjection('XY', 3) * dir
    # API Change in 2.59 builds - Vectors are now column vectors
    
    ray.range = 0.8/xy.length
