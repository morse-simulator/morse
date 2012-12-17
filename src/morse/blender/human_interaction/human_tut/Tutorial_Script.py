from morse.core import blenderapi
import bgl
import blf
import parse_file


co = blenderapi.controller()
ow = co.owner

mainScene = blenderapi.scene()

font_id = 0

def write():
    """write on screen"""
    width = blenderapi.render().getWindowWidth()
    height = blenderapi.render().getWindowHeight()

    # OpenGL setup
    bgl.glMatrixMode(bgl.GL_PROJECTION)
    bgl.glLoadIdentity()
    bgl.gluOrtho2D(0, width, 0, height)
    bgl.glMatrixMode(bgl.GL_MODELVIEW)
    bgl.glLoadIdentity()

    # BLF drawing routine
    blf.size(font_id, height//40, 72)
    data = parse_file.read_file()
    data = data.splitlines()
    linePosition = height * 0.8
    for str in data:
        str_len = len(str)
        blf.position(font_id, (width * 0.05), linePosition, 0)
        blf.enable(font_id, blf.SHADOW)
        blf.shadow(font_id, 0, 1.0, 0.2, 0.0, 1.0)
        blf.draw(font_id,str)
        linePosition -= height * 0.05

def init():
    """init function - runs once"""
    # set the font drawing routine to run every frame
    mainScene.post_draw = [write]

        
