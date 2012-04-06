import logging; logger = logging.getLogger("morse." + __name__)

import os

from bge import logic, render, texture
import bgl, blf
from mathutils import Matrix, Vector

from bpy import data

from morse.helpers import passive_objects

font_id = 0

windowWidth = render.getWindowWidth()
windowHeight = render.getWindowHeight()
scene= logic.getCurrentScene()
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



def interact():
    """
    Script for opening doors, drawers and grabbing objects
    
    press left mousebutton to open, close or grab
    press right mousebutton to drop the currently selected object
    """

    right_hand=objects['IK_Target_Empty.R']
    look = objects['Target_Empty']
    human = objects['Human']
    co =  logic.getCurrentController()
    ow = co.owner
    lmb=co.sensors['LMB']
    ray = co.sensors['Ray']
    cam = ray.owner
    lay_down_ray = co.sensors['LayDownRay']
    rmb=co.sensors['RMB']
    space = co.sensors['SPACEBAR']
    head = objects['Head_Empty']
    hand = objects['Hand.R']

    
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
            actor_focused = data.objects[focused_object.name].game.use_actor
        # accurate placing of objects under certain conditions
        if human['Manipulate'] and lay_down_ray.positive \
           and focused_object != ow['selected'] \
           and actor_focused:
            # check not to lay the object on itself
            if ow['selected']:
                right_hand['LayDown'] = lay_down_ray.hitPosition
        # otherwise just drop the object
        else:
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

    right_hand = objects['IK_Target_Empty.R']
    human = objects['Human']
    hand = objects['Hand.R']
    hips = objects['Hips_Empty']
    left_hand = objects['IK_Target_Empty.L']

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

def lay_down():
    """
    lay the object down to given coordinates
    """
    co = logic.getCurrentController()
    pos = co.owner
    objects = logic.getCurrentScene().objects
    hand = objects['Hand_Grab.R']
    
    obj = hand['selected']
    if obj == None or not pos['LayDown']:
        return

    vect = pos.getVectTo(Vector(pos['LayDown']))[1]
    
    obj_collision = obj.sensors['Collision']
    
    if not obj_collision.positive:
        pos.worldPosition += vect/75
    else:
        obj.removeParent()
        hand['selected'] = None
        pos['moveArm'] = True
        pos['LayDown'] = False

def lay_down_visualize():
    """
    Show a green rectangle if you can accurately place the selected object
    """
    co = logic.getCurrentController()
    ow = co.owner
    scene = logic.getCurrentScene()
    objects = scene.objects
    human = objects['Human']
    hand = objects['Hand_Grab.R']
    ray = co.sensors['LayDownRay']


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
    hand = objects['Hand_Grab.R']
    
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


    hand = objects['Hand_Grab.R']

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

