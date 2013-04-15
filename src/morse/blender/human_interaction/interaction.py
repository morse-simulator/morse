import logging; logger = logging.getLogger("morse." + __name__)

import os

from bge import logic, texture
import bgl, blf
from mathutils import Matrix, Vector

from bpy import data

from morse.core import blenderapi
from morse.helpers import passive_objects

font_id = 0

windowWidth = blenderapi.render().getWindowWidth()
windowHeight = blenderapi.render().getWindowHeight()
scene= blenderapi.scene()
objects = scene.objects

texco=[(0,0), (1,0), (1,1), (0,1)]

def loadtexture(filepath):
    """ Loads a texture from an image (tga, jpg...any format supported by FFMPEG)
    and returns the texture buffer ID.
    """

    id_buf = bgl.Buffer(bgl.GL_INT, 1)
    bgl.glGenTextures(1, id_buf)
    id = id_buf.to_list()[0] if hasattr(id_buf, "to_list") else id_buf.list[0]
    bgl.glBindTexture(bgl.GL_TEXTURE_2D, id)
    image = texture.ImageFFmpeg(filepath)
    if not image.image:
        logger.error("Error when loading " + filepath + ". File not found? Format not "
                     "supported by FFMPEG? (tga, jpg, png do work)")
        return -1
    else:
        im_buf = image.image
        bgl.glTexParameteri(bgl.GL_TEXTURE_2D, bgl.GL_TEXTURE_MAG_FILTER, bgl.GL_LINEAR)
        bgl.glTexParameteri(bgl.GL_TEXTURE_2D, bgl.GL_TEXTURE_MIN_FILTER, bgl.GL_LINEAR)
        bgl.glTexEnvf(bgl.GL_TEXTURE_ENV, bgl.GL_TEXTURE_ENV_MODE, bgl.GL_MODULATE)
        bgl.glTexImage2D(bgl.GL_TEXTURE_2D, 0, bgl.GL_RGBA, image.size[0], image.size[1], 0, bgl.GL_RGBA, bgl.GL_UNSIGNED_BYTE, im_buf)
        return id


# load overlay_closed.tga and overlay_open.tga into the global dictionary
if not "open" in  logic.globalDict:
    TexName = "overlay_open.tga"
    filepath = logic.expandPath(os.path.join(os.environ["MORSE_ROOT"], "share","morse","data","props",TexName))
    logic.globalDict["open"] = loadtexture(filepath)

open_id = logic.globalDict.get("open")

if not "closed" in logic.globalDict:
    TexName = "overlay_closed.tga"
    filepath = os.path.join(os.environ["MORSE_ROOT"], "share","morse","data","props",TexName)
    logic.globalDict["closed"] = loadtexture(filepath)

closed_id = logic.globalDict.get("closed")

# load CrossHairs.tga into the global dictionary
if not "crosshairs" in logic.globalDict:
    TexName = "overlay_crosshairs.tga"
    filepath = os.path.join(os.environ["MORSE_ROOT"], "share","morse","data","props",TexName)
    logic.globalDict["crosshairs"] = loadtexture(filepath)

crosshairs_id = logic.globalDict.get("crosshairs")


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



def interact(cont):
    """
    Script for opening doors, drawers and grabbing objects
    
    press left mousebutton to open, close or grab
    press right mousebutton to drop the currently selected object
    """

    ow = cont.owner

    # get the suffix of the human to reference the right objects
    suffix = ow.name[-4:] if ow.name[-4] == "." else ""

    right_hand=objects['IK_Target_Empty.R' + suffix]
    look = objects['Target_Empty' + suffix]
    human = blenderapi.scene().objects[ow.parent.parent.parent.name]

    # if the human is external, do nothing
    if human.get('External_Robot_Tag') or human['disable_keyboard_control']:
        return
    
    lmb=cont.sensors['LMB']
    ray = cont.sensors['Ray']
    cam = ray.owner
    lay_down_ray = cont.sensors['LayDownRay']
    rmb=cont.sensors['RMB']
    space = cont.sensors['SPACEBAR']
    head = objects['Head_Empty' + suffix]
    hand = objects['Hand.R' + suffix]

    
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
        elif 'Door' in prox_obj or 'Drawer' in prox_obj or 'Switch' in prox_obj:
            focus = prox_obj
        else:
            for obj in prox_obj.children:
                if 'Object' in obj:
                    focus = obj
    
    # set the overlay scene and change the displayed text
    # and texture
    if human['Manipulate'] and focus:
            
        can_be_manipulated = False

        if focus in passive_objects.graspable_objects():
            can_be_manipulated = True
            if not ow['selected']:
                ow['Status'] = 'Pick up ' + passive_objects.label(focus)
            else:
                ow['Status'] = passive_objects.label(focus)
        elif 'Door' in focus or 'Drawer' in focus:
            can_be_manipulated = True
            
            try:
                if focus['Open']:
                    ow['Status'] = 'Close ' + str(focus['Description'])
                else:
                    ow['Status'] = 'Open ' + str(focus['Description'])
            except KeyError:
                logger.warning('Key missing in focused Object ' + focus.name +
                      ' --- no description given')
        elif 'Switch' in focus:
            can_be_manipulated = True
            if objects[focus['Switch']]['On']:
                ow['Status'] = "Turn off " + focus['Switch']
            else:
                ow['Status'] = "Turn on " + focus['Switch']
        else:
            ow['Status'] = None
    else:
        ow['Status'] = None

    if human['Manipulate']:
        if not crosshairs in scene.post_draw:
            scene.post_draw.append(crosshairs)
    else:
        if crosshairs in scene.post_draw:
            scene.post_draw.remove(crosshairs)


    if ow['Status']:
        if not write_interaction_status in scene.post_draw:
            scene.post_draw.append(write_interaction_status)
        if not status_image in scene.post_draw:
            scene.post_draw.append(status_image)
    else:
        if write_interaction_status in scene.post_draw:
            scene.post_draw.remove(write_interaction_status)
        if status_image in scene.post_draw:
            scene.post_draw.remove(status_image)



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
        elif 'Switch' in focus and lmb.positive:
            objects[focus['Switch']]['On'] = not objects[focus['Switch']]['On']
    except TypeError:
        pass


    if rmb.positive:                #drop selected Object
        ow['grabbing'] = None
        focused_object = lay_down_ray.hitObject
        if focused_object != None:
            actor_focused = blenderapi.objectdata(focused_object.name).game.use_actor
        # accurate placing of objects under certain conditions
        if human['Manipulate'] and lay_down_ray.positive \
           and focused_object != ow['selected'] \
           and actor_focused:
            # check not to lay the object on itself
            if ow['selected']:
                right_hand['LayDown'] = lay_down_ray.hitPosition
                right_hand['LayDownObj'] = focused_object
        # otherwise just drop the object
        else:
            if ow['selected']:
                ow['selected'].removeParent()
                ow['selected'] = None
                right_hand['moveArm'] = True



def grab(cont):
    """
    Makes the right hand move to the object
    and parent the object to the hand if colliding
    """
    ow = cont.owner

    # get the suffix of the human to reference the right objects
    suffix = ow.name[-4:] if ow.name[-4] == "." else ""

    # Nothing selected for grabbing or already something in hand
    if not ow['grabbing'] or ow['selected']:
        return

    obj = ow['grabbing']

    coll = cont.sensors['Collision']

    human = objects[ow.parent.parent.parent.name + suffix]
    hand = objects['Hand.R' + suffix]
    hips = objects['Hips_Empty' + suffix]
    left_hand = objects['IK_Target_Empty.L' + suffix]
    right_hand = objects['IK_Target_Empty.R' + suffix]

    vect = right_hand.getVectTo(obj)
    move = vect[1]
    right_hand.applyMovement(move/50)

    if obj.worldPosition[2] < human.worldPosition[2] + 0.5:
        # if the object is located lower than 0.5 meters (local Position)
        hips.applyMovement([0.0, 0.0, -(hips.worldPosition[2]-0.5)/50])
        left_hand.applyMovement([0.0, 0.0, -(hips.worldPosition[2]-0.5)/50])

    # Do we actually touch the object?
    # Even if the physics shape consists of several compound objects,
    # the parent is considered when one of them is touched

    if coll.hitObject == obj:
        logger.debug("Grabbing %s" % obj)
        ow['grabbing'] = None
        ow['selected'] = obj
        obj.setParent(ow)
        right_hand['moveArm'] = True

def roll_hand_r(cont):
    """
    roll the right hand
    """
    armature = cont.owner
    human = armature.parent
    hand_r = armature.channels['Hand.R']

    # if the human is external, do nothing
    if human.get('External_Robot_Tag') or human['disable_keyboard_control']:
        return
    
    keyboard = cont.sensors['All_Keys']
    wheel_down = cont.sensors['WheelDown']
    wheel_up = cont.sensors['WheelUp']

    if not ((wheel_down.positive or wheel_up.positive) and human['Manipulate']):
        return
    
    roll_speed = 0.1
    
    keylist = keyboard.events
    for key in keylist:
        # key[0] == events.keycode, key[1] = status
        if key[1] == blenderapi.input_active() and key[0] == blenderapi.LEFTCTRLKEY:
            if wheel_down.positive:
                roll_speed = -roll_speed
            hand_r.rotation_quaternion += Vector((0.0, 0.0, roll_speed, 0.0))
            armature.update()
            


def lay_down(cont):
    """
    lay the object down to given coordinates
    """
    pos = cont.owner

    # get the suffix of the human to reference the right objects
    suffix = pos.name[-4:] if pos.name[-4] == "." else ""
    
    objects = blenderapi.scene().objects
    hand = objects['Hand_Grab.R' + suffix]
    
    obj = hand['selected']
    if obj == None or not pos['LayDown']:
        return

    vect = pos.getVectTo(Vector(pos['LayDown']))[1]
    
    obj_collision = obj.sensors['Collision']
    
    if not (obj_collision.positive and pos['LayDownObj'] in obj_collision.hitObjectList):
        pos.worldPosition += vect/75
    else:
        obj.removeParent()
        hand['selected'] = None
        pos['moveArm'] = True
        pos['LayDown'] = False
        pos['LayDownObj'] = None

def lay_down_visualize(cont):
    """
    Show a green rectangle if you can accurately place the selected object
    """
    ow = cont.owner

    # get the suffix of the human to reference the right objects
    suffix = ow.name[-4:] if ow.name[-4] == "." else ""
    
    scene = blenderapi.scene()
    objects = scene.objects
    human = objects[ow['human_name']]

    hand = objects['Hand_Grab.R' + suffix]
    ray = cont.sensors['LayDownRay']


    focused_object = ray.hitObject
    try:
        actor_focused = data.objects[focused_object.name].game.use_actor
    except AttributeError:
        actor_focused = False
        
    if human['Manipulate'] and ray.positive and \
       focused_object != hand['selected'] and hand['selected'] and \
       actor_focused:
        if not color_placing in scene.post_draw:
            scene.post_draw.append(color_placing)
            
    else:
        if color_placing in scene.post_draw:
            scene.post_draw.remove(color_placing)


def color_placing():
    """
    Draw the green rectangle via OpenGL-Wrapper
    """
    imageHeight = windowHeight * 0.05
    imageWidth = imageHeight

    x = windowWidth * 0.5 - imageWidth/2
    y = windowHeight * 0.5 - imageHeight/2
    
    gl_position = [[x, y],[x+imageWidth, y],[x+imageWidth, y+imageHeight],[x, y+imageHeight]]
    
    view_buf = bgl.Buffer(bgl.GL_INT, 4)
    bgl.glGetIntegerv(bgl.GL_VIEWPORT, view_buf)
    view = view_buf.to_list() if hasattr(view_buf, "to_list") else view_buf.list
    # Save the state
    bgl.glPushAttrib(bgl.GL_ALL_ATTRIB_BITS)      
    # Disable depth test so we always draw over things
    bgl.glDisable(bgl.GL_DEPTH_TEST)   
    # Disable lighting so everything is shadless
    bgl.glDisable(bgl.GL_LIGHTING)   
    # Make sure we're using smooth shading instead of flat
    bgl.glShadeModel(bgl.GL_SMOOTH)
    # Setup the matrices
    bgl.glMatrixMode(bgl.GL_PROJECTION)
    bgl.glPushMatrix()
    bgl.glLoadIdentity()
    bgl.gluOrtho2D(0, view[2], 0, view[3])
    bgl.glMatrixMode(bgl.GL_MODELVIEW)
    bgl.glPushMatrix()
    bgl.glLoadIdentity()
    
    
    # Enable alpha blending
    bgl.glEnable(bgl.GL_BLEND)
    bgl.glBlendFunc(bgl.GL_SRC_ALPHA, bgl.GL_ONE_MINUS_SRC_ALPHA)
    # Draw the colored quad
    bgl.glColor4f(0, 1, 0, 0.25)
    bgl.glEnable(bgl.GL_POLYGON_OFFSET_FILL)
    bgl.glPolygonOffset(1.0, 1.0)
    bgl.glBegin(bgl.GL_QUADS)
    for i in range(4):
       bgl.glTexCoord2f(texco[i][0], texco[i][1])
       bgl.glVertex2f(gl_position[i][0], gl_position[i][1])
    bgl.glEnd()

    bgl.glPopMatrix()
    bgl.glMatrixMode(bgl.GL_PROJECTION)
    bgl.glPopMatrix()
    bgl.glPopAttrib()

    
def write_interaction_status():
    """
    Write the interaction status on Screen
    The status is stored in a property
    """
    cam = blenderapi.scene().active_camera

    # get the suffix of the human to reference the right objects
    suffix = cam.name[-4:] if cam.name[-4] == "." else ""
    
    hand = objects['Hand_Grab.R' + suffix]
    
    # OpenGL setup
    bgl.glMatrixMode(bgl.GL_PROJECTION)
    bgl.glLoadIdentity()
    bgl.gluOrtho2D(0, windowWidth, 0, windowHeight)
    bgl.glMatrixMode(bgl.GL_MODELVIEW)
    bgl.glLoadIdentity()
    
    blf.size(font_id, int(windowHeight*0.04), 72)
    # draw a black shadow around the text
    blf.enable(font_id, blf.SHADOW)
    blf.shadow(font_id, 5, 0.0, 0.0, 0.0, 1.0)
    blf.position(font_id, windowWidth*0.4, windowHeight*0.4,0)
    blf.draw(font_id, hand['Status'])

def status_image():
    """
    Show the corrensponding Image for the status
    """
    imageHeight = windowHeight * 0.075
    imageWidth = imageHeight

    x = windowWidth * 0.35 - imageWidth/2
    y = windowHeight * 0.45 - imageHeight/2
    
    gl_position = [[x, y],[x+imageWidth, y],[x+imageWidth, y+imageHeight],[x, y+imageHeight]]

    cam = blenderapi.scene().active_camera

    # get the suffix of the human to reference the right objects
    suffix = cam.name[-4:] if cam.name[-4] == "." else ""

    hand = objects['Hand_Grab.R' + suffix]

    # select the right Image
    if hand["selected"]:
        tex_id = closed_id
    else:
        tex_id = open_id

    
    view_buf = bgl.Buffer(bgl.GL_INT, 4)
    bgl.glGetIntegerv(bgl.GL_VIEWPORT, view_buf)
    view = view_buf.to_list() if hasattr(view_buf, "to_list") else view_buf.list
    # Save the state
    bgl.glPushAttrib(bgl.GL_ALL_ATTRIB_BITS)      
    # Disable depth test so we always draw over things
    bgl.glDisable(bgl.GL_DEPTH_TEST)   
    # Disable lighting so everything is shadless
    bgl.glDisable(bgl.GL_LIGHTING)   
    # Unbinding the texture prevents BGUI frames from somehow picking up on
    # color of the last used texture
    bgl.glBindTexture(bgl.GL_TEXTURE_2D, 0)      
    # Make sure we're using smooth shading instead of flat
    bgl.glShadeModel(bgl.GL_SMOOTH)
    # Setup the matrices
    bgl.glMatrixMode(bgl.GL_PROJECTION)
    bgl.glPushMatrix()
    bgl.glLoadIdentity()
    bgl.gluOrtho2D(0, view[2], 0, view[3])
    bgl.glMatrixMode(bgl.GL_MODELVIEW)
    bgl.glPushMatrix()
    bgl.glLoadIdentity()
    
    
    bgl.glEnable(bgl.GL_TEXTURE_2D)
    # Enable alpha blending
    bgl.glEnable(bgl.GL_BLEND)
    bgl.glBlendFunc(bgl.GL_SRC_ALPHA, bgl.GL_ONE_MINUS_SRC_ALPHA)
    # Bind the texture
    bgl.glBindTexture(bgl.GL_TEXTURE_2D, tex_id)
    # Draw the textured quad
    bgl.glColor4f(1, 1, 1, 1)
    bgl.glEnable(bgl.GL_POLYGON_OFFSET_FILL)
    bgl.glPolygonOffset(1.0, 1.0)
    bgl.glBegin(bgl.GL_QUADS)
    for i in range(4):
       bgl.glTexCoord2f(texco[i][0], texco[i][1])
       bgl.glVertex2f(gl_position[i][0], gl_position[i][1])
    bgl.glEnd()
    bgl.glBindTexture(bgl.GL_TEXTURE_2D, 0)

    bgl.glPopMatrix()
    bgl.glMatrixMode(bgl.GL_PROJECTION)
    bgl.glPopMatrix()
    bgl.glPopAttrib()

def crosshairs():
    """
    Show crosshais in Manipulation Mode
    Use the OpenGL-Wrapper to draw the image
    """
    
    imageHeight = windowHeight * 0.05
    imageWidth = imageHeight

    x = windowWidth * 0.5 - imageWidth/2
    y = windowHeight * 0.5 - imageHeight/2
    
    gl_position = [[x, y],[x+imageWidth, y],[x+imageWidth, y+imageHeight],[x, y+imageHeight]]
    
    view_buf = bgl.Buffer(bgl.GL_INT, 4)
    bgl.glGetIntegerv(bgl.GL_VIEWPORT, view_buf)
    view = view_buf.to_list() if hasattr(view_buf, "to_list") else view_buf.list
    # Save the state
    bgl.glPushAttrib(bgl.GL_ALL_ATTRIB_BITS)      
    # Disable depth test so we always draw over things
    bgl.glDisable(bgl.GL_DEPTH_TEST)   
    # Disable lighting so everything is shadless
    bgl.glDisable(bgl.GL_LIGHTING)   
    # Unbinding the texture prevents BGUI frames from somehow picking up on
    # color of the last used texture
    bgl.glBindTexture(bgl.GL_TEXTURE_2D, 0)      
    # Make sure we're using smooth shading instead of flat
    bgl.glShadeModel(bgl.GL_SMOOTH)
    # Setup the matrices
    bgl.glMatrixMode(bgl.GL_PROJECTION)
    bgl.glPushMatrix()
    bgl.glLoadIdentity()
    bgl.gluOrtho2D(0, view[2], 0, view[3])
    bgl.glMatrixMode(bgl.GL_MODELVIEW)
    bgl.glPushMatrix()
    bgl.glLoadIdentity()
    
    bgl.glEnable(bgl.GL_TEXTURE_2D)
    # Enable alpha blending
    bgl.glEnable(bgl.GL_BLEND)
    bgl.glBlendFunc(bgl.GL_SRC_ALPHA, bgl.GL_ONE_MINUS_SRC_ALPHA)
    # Bind the texture
    bgl.glBindTexture(bgl.GL_TEXTURE_2D, crosshairs_id)
    # Draw the textured quad
    bgl.glColor4f(1, 1, 1, 1)
    bgl.glEnable(bgl.GL_POLYGON_OFFSET_FILL)
    bgl.glPolygonOffset(1.0, 1.0)
    bgl.glBegin(bgl.GL_QUADS)
    for i in range(4):
       bgl.glTexCoord2f(texco[i][0], texco[i][1])
       bgl.glVertex2f(gl_position[i][0], gl_position[i][1])
    bgl.glEnd()
    bgl.glBindTexture(bgl.GL_TEXTURE_2D, 0)

    bgl.glPopMatrix()
    bgl.glMatrixMode(bgl.GL_PROJECTION)
    bgl.glPopMatrix()
    bgl.glPopAttrib()

