
from bge import logic
from mathutils import Matrix, Vector

def opendoor(door):
    if lmb.positive and door['Open']==False:     #opens the door
        if door['Door']=='right':
            door.applyRotation((0, 0, 1.4), False)       #rotation around global Z-Axis - ~80 degrees
        elif door['Door']=='left':
            door.applyRotation((0, 0, -1.4), False)
        door['Open']=True
    elif lmb.positive and door['Open']==True:     #closes the door
        if door['Door']=='right':
            door.applyRotation((0, 0, -1.4), False)      #rotation around global Z-Axis - ~80 degrees
        elif door['Door']=='left':
            door.applyRotation((0, 0, 1.4), False)
        door['Open']=False

"""
def opendrawer(drawer):                     #function for drawers --> not in use (using IPOs now)
    if lmb.positive and drawer['Open']==False:     #opens the drawer
        #drawer.applyMovement((0, -drawer.worldScale[2]*0.7, 0), True)       #((0,-objectDepth,0), local)        !!IPOs for better looking drawers!!!
        drawer['Open']=True
    elif lmb.positive and drawer['Open']==True:     #closes the drawer
        #drawer.applyMovement((0, drawer.worldScale[2]*0.7, 0), True)
        drawer['Open']=False
"""

"""
def grabobject(obj):                    #function for grabbing objects --> not in use
    smoothness = 20
    currLoc = right_hand.worldPosition
    destLoc = obj.worldPosition
    dX, dY, dZ = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    for x in range(0, smoothness):      #hand moves to object until it collides

        if touch.hitObject == obj:
            focus.setParent(ow)
            right_hand['moveArm'] = True
        else:
            dX = currLoc[0] - destLoc[0]        
            dY = currLoc[1] - destLoc[1]        
            dZ = currLoc[2] - destLoc[2]
            smoothVec = currLoc                 
            smoothVec[0] = smoothVec[0] - dX/smoothness         
            smoothVec[1] = smoothVec[1] - dY/smoothness         
            smoothVec[2] = smoothVec[2] - dZ/smoothness
            right_hand.worldPosition()   
                
        
    ow['selected'] = obj        #needed for droping the object
"""

def interact():
    """
    Script for opening doors, drawers and grabbing objects
    
    press left mousebutton to open, close or grab
    press right mousebutton to drop the currently selected object
    """
    sobList= logic.getCurrentScene().objects     #list of all objects in current scene
    right_hand=sobList['IK_Target_Empty.R']
    look = sobList['Target_Empty']
    human = sobList['POS_EMPTY']
    co =  logic.getCurrentController()
    ow = co.owner
    lmb=co.sensors['LMB']
    ray = co.sensors['Ray']
    cam = ray.owner
    rmb=co.sensors['RMB']
    space = co.sensors['SPACEBAR']
    AddSc = co.actuators['AddScene']
    RemSc = co.actuators['RemoveScene']
    Description = co.actuators['Descr_message']
    head = sobList['Head_Empty']
    hand = sobList['Hand.R']
    
    focus = ray.hitObject                     #focused object
    
    scenes =  logic.getSceneList()
    if human['Manipulate']:
        try:
            if 'Door' in focus or 'Object' in focus or 'Drawer' in focus:# or (ow['selected'] != 'None' and ow['selected'] != ''):
                try:
                    if 'Object' in focus:
                        Description.body = 'Pick up ' + str(focus['Description'])
                    else:
                        if focus['Open'] == True:
                            Description.body = 'Close ' + str(focus['Description'])
                        else:
                            Description.body = 'Open ' + str(focus['Description'])
                except KeyError:
                    print('Key missing in focused Object ' + focus.name + ' --- no description given')
                    Description.body = ''
                ow.sendMessage('selected', str(ow['selected']), 'overlay')
                co.activate(Description)
                if len(scenes) == 2:
                    co.activate(AddSc)
            else:
                try:
                    scenes[2]
                    co.activate(RemSc)
                except IndexError:
                    pass
        except TypeError:
            try:
                scenes[2]
                co.activate(RemSc)
            except IndexError:
                pass
    else:
        try:
            scenes[2]
            co.activate(RemSc)
        except IndexError:
            pass



    if space.positive:            # blocks mouse movement if interactable object is focused 
        try:
            if 'Door' in focus or 'Object' in focus or 'Drawer' in focus:
                human['FOCUSED'] = True
                try:
                    vect = human.getVectTo(focus)[1] * Matrix.OrthoProjection('XY', 3)
                except ValueError:
                    vect = Matrix.OrthoProjection('XY', 3) * human.getVectTo(focus)[1]  # API Change in unofficial 2.59 builds - Vectors are now column vectors
                human.alignAxisToVect(vect, 0, 1.0)
            else:
                human['FOCUSED'] = False
        except TypeError:
            human['FOCUSED'] = False
    else:
        human['FOCUSED'] = False


    try:
        if 'Object' in focus:
            if lmb.positive and (ow['selected'] == 'None' or ow['selected'] == ''):
                #grabObject(focus)
                ow['grabbing'] = focus
        elif 'Door' in focus:
            openDoor(focus)
        elif 'Drawer' in focus and lmb.positive:
            #openDrawer(focus)
            focus['Open'] = not focus['Open']
    except TypeError:
        pass


    if rmb.positive:                #drop selected Object
        ow['grabbing'] = 'NONE'
        try:
            if ow['selected'] != 'None' and ow['selected'] != '':
                selected = ow['selected']
                selected.removeParent()
                ow['selected'] = 'None'
                if human ['Manipulate']:
                    right_hand.localPosition = [0.6, 0.0, 1.4]
                    look.worldPosition = right_hand.worldPosition
                    head.worldOrientation = human.worldOrientation
                    #hand.alignAxisToVect(([0.0, 1.0, 0.0]*human.worldOrientation), 2, 1.0)
                    #hand.alignAxisToVect(-hand.getVectTo(human)[1], 2, 1.0)
                    # reset head.localOrientation
                else:
                    right_hand.localPosition = [0.0, -0.3, 0.8]
                    look.localPosition = [0.0, 0.3, 0.8]
                    head.worldOrientation = human.worldOrientation
            else:
                right_hand['moveArm'] = True
        except TypeError:
            pass


def grab():
    co = logic.getCurrentController()
    ow = co.owner
    coll = co.sensors['Collision']
    sobList = logic.getCurrentScene().objects
    right_hand = sobList['IK_Target_Empty.R']
    human = sobList['Human']
    hand = sobList['Hand.R']
    hips = sobList['Hips_Empty']
    left_hand = sobList['IK_Target_Empty.L']
    
    if ow['grabbing'] != 'NONE' and (ow['selected'] == 'None' or ow['selected'] == ''):
        dest = sobList[str(ow['grabbing'])]
    
        vect = right_hand.getVectTo(dest)
        
        move = vect[1]
        '''
        test = vect[1].project(Vector([1.0, 0.0, 0.0])*human.worldOrientation)
        
        if test.dot(test) > 0.1**2:
            move += Vector([0.0, -1.0, 0.0])*human.worldOrientation
        '''
        
        right_hand.applyMovement(move/50)
        #hand.alignAxisToVect(-test, 1, 1.0)
        #hand.alignAxisToVect([0.0, 0.0, 1.0], 0, 1.0)
        if dest.worldPosition[2] < human.worldPosition[2] + 0.5:
            hips.applyMovement([0.0, 0.0, -(hips.worldPosition[2]-0.5)/50])
            left_hand.applyMovement([0.0, 0.0, -(hips.worldPosition[2]-0.5)/50])
        
        if coll.hitObject == dest:
        # right_hand.worldPosition = dest.worldPosition
            ow['grabbing'] = 'NONE'
            ow['selected'] = dest
            ow.sendMessage('selected', str(ow['selected']), 'overlay')
            ow.sendMessage('Description', 'Pick up ' + str(dest['Description']), 'Description')
            dest.setParent(ow)
            right_hand['moveArm'] = True
    
