import logging; logger = logging.getLogger("morse." + __name__)

from morse.core import blenderapi
import bgl
import blf
import bge

from morse.helpers import passive_objects

font_id =0

co = blenderapi.controller()
keyboard = co.sensors['All_Keys']

scene = blenderapi.scene()

windowWidth = blenderapi.render().getWindowWidth()
windowHeight = blenderapi.render().getWindowHeight()

def write():
    """
    Write the name of all active objects on Screen
    """
    # OpenGL setup
    bgl.glMatrixMode(bgl.GL_PROJECTION)
    bgl.glLoadIdentity()
    bgl.gluOrtho2D(0, windowWidth, 0, windowHeight)
    bgl.glMatrixMode(bgl.GL_MODELVIEW)
    bgl.glLoadIdentity()
    
    cam = scene.active_camera

    for obj in passive_objects.active_objects():
        # test if the object is in the view frustum
        if cam.pointInsideFrustum(obj.worldPosition):
            pos = cam.getScreenPosition(obj)
    
            blf.size(font_id, int(windowWidth * 0.02), 72)
            # draw a black shadow to increase contrast with white parts
            blf.enable(font_id, blf.SHADOW)
            blf.shadow(font_id, 5, 0.0, 0.0, 0.0, 1.0)
            blf.position(font_id, pos[0]*windowWidth, (1 - pos[1])*windowHeight,0)
            blf.draw(font_id, obj.name)


def test(contr):
    """
    Show which objects are interactable
    """
    keylist = keyboard.events

    for key in keylist:
        if key[0] == bge.events.LEFTALTKEY:
            if key[1] == blenderapi.input_just_activated():
                #show text over all objects
                scene.post_draw.append(write)
                
            elif key[1] == blenderapi.input_just_released():
                #hide text of all objects
                scene.post_draw.remove(write)
                

