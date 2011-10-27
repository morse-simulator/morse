import logging; logger = logging.getLogger("morse." + __name__)
from bge import logic
from mathutils import Matrix, Vector

from morse.helpers import passive_objects

def open_door(door):
    if not door['Open']:     # opens the door
        if door['Door'].lower()=='right':
            door.applyRotation((0, 0, 1.4), False)
            # rotation around global Z-Axis - ~80 degrees
        elif door['Door'].lower()=='left':
            door.applyRotation((0, 0, -1.4), False)
    elif door['Open']:     # closes the door
        if door['Door'].lower()=='right':
            door.applyRotation((0, 0, -1.4), False)
            # rotation around global Z-Axis - ~80 degrees
        elif door['Door'].lower()=='left':
            door.applyRotation((0, 0, 1.4), False)
    door['Open'] = not door['Open']



def interact():
    """
    Script for opening doors, drawers and grabbing objects
    
    press left mousebutton to open, close or grab
    press right mousebutton to drop the currently selected object
    """
    sobList= logic.getCurrentScene().objects
    # list of all objects in current scene
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
   
    # Get the focusing object:
    # A ray sensor is attached to the HumanCamera sensor.
    # It returns all colliding objects in a 10 cm range of the hand.
    # We filter the result to keep only objects that have the 'Object'
    # property or that have children with the 'Object' property.
    focus = None
    prox_obj = ray.hitObject                     # focused object
    if prox_obj:
        if 'Object' in prox_obj:
            focus = prox_obj
        else:
            for obj in prox_obj.children:
                if 'Object' in obj:
                    focus = obj
    
    scenes =  logic.getSceneList()
    
    # set the overlay scene and send messages to change the displayed text
    # and texture
    if human['Manipulate']:
        try:

            can_be_manipulated = False

            if focus in passive_objects.graspable_objects():
                can_be_manipulated = True
                Description.body = ('Pick up ' + passive_objects.label(focus))

            elif 'Door' in focus or 'Drawer' in focus:
                can_be_manipulated = True
                try:
                    if focus['Open']:
                        Description.body = ('Close ' +
                                            str(focus['Description']))
                    else:
                        Description.body = ('Open ' +
                                            str(focus['Description']))
                except KeyError:
                    print('Key missing in focused Object ' + focus.name +
                          ' --- no description given')
                    Description.body = ''

            if can_be_manipulated:
                ow.sendMessage('selected', str(ow['selected']), 'overlay')
                co.activate(Description)
                if len(scenes) == 2:
                    co.activate(AddSc)

            else:
                if len(scenes) == 3:
                    co.activate(RemSc)
        except TypeError:
            if len(scenes) == 3:
                    co.activate(RemSc)
    else:
        if len(scenes) == 3:
                    co.activate(RemSc)



    if space.positive:
        # blocks mouse movement if interactable object is focused 
        try:
            if ('Door' in focus or 'Object' in focus or 'Drawer' in focus) and not ow['selected']:

                human['FOCUSED'] = True
                vect = Matrix.OrthoProjection('XY', 3) * human.getVectTo(focus)[1]
                human.alignAxisToVect(vect, 0, 1.0)
                # align the local x axis to point to the focused object
            else:
                human['FOCUSED'] = False
        except TypeError:
            human['FOCUSED'] = False
    else:
        human['FOCUSED'] = False


    try:
        if focus in passive_objects.graspable_objects():
            if lmb.positive and not ow['selected']:
                # set a property - a property-sensor will fire the grab-function
                ow['grabbing'] = focus
        elif 'Door' in focus and lmb.positive:
            open_door(focus)
            # if you decide to use IPOs for the doors,
            # comment the previous line and uncomment the next line
            # the logic can be set with code in morse utils, which is currently
            # commented
            # focus['Open'] = not focus['Open']
        elif 'Drawer' in focus and lmb.positive:
            focus['Open'] = not focus['Open']
    except TypeError:
        pass


    if rmb.positive:                #drop selected Object
        ow['grabbing'] = None
        if ow['selected']:
            ow['selected'].removeParent()
            ow['selected'] = None
            right_hand['moveArm'] = True



def grab():
    """
    Makes the right hand move to the object
    and parent the object to the hand if colliding
    """
    co = logic.getCurrentController()
    ow = co.owner

    # Nothing selected for grabbing or already something in hand
    if not ow['grabbing'] or ow['selected']:
        return

    obj = ow['grabbing']

    coll = co.sensors['Collision']
    sobList = logic.getCurrentScene().objects
    right_hand = sobList['IK_Target_Empty.R']
    human = sobList['Human']
    hand = sobList['Hand.R']
    hips = sobList['Hips_Empty']
    left_hand = sobList['IK_Target_Empty.L']

    vect = right_hand.getVectTo(obj)
    move = vect[1]
    right_hand.applyMovement(move/50)

    if obj.worldPosition[2] < human.worldPosition[2] + 0.5:
        # if the object is located lower than 0.5 meters (local Position)
        hips.applyMovement([0.0, 0.0, -(hips.worldPosition[2]-0.5)/50])
        left_hand.applyMovement([0.0, 0.0, -(hips.worldPosition[2]-0.5)/50])

    # Do we actually touch the object?
    if len(obj.meshes) == 0: # Most probably an EMPTY: check children's meshes.
        objs = obj.children
    else:
        objs = [obj]

    for subobj in objs:
        if coll.hitObject == subobj:
            logger.debug("Grabbing %s" % obj)
            ow['grabbing'] = None
            ow['selected'] = obj
            obj.setParent(ow)
            right_hand['moveArm'] = True
            break

