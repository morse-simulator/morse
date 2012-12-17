from morse.core import blenderapi

def RGBtoHue(rgbList):
    """ Convert an RGB color to Hue """
    R = rgbList[0]
    G = rgbList[1]
    B = rgbList[2]
    
    # min, max, delta;
    min_rgb = min( R, G, B )
    max_rgb = max( R, G, B )
    delta = max_rgb - min_rgb

    if not delta:
        return 0
    if R == max_rgb:
        H = ( G - B ) / delta # between yellow & magenta
    elif G == max_rgb:
        H = 2 + ( B - R ) / delta # between cyan & yellow
    else:
        H = 4 + ( R - G ) / delta # between magenta & cyan
    H *= 60 # convert to deg
    if H < 0:
        H += 360
    return int(H)


def retrieveHue(obj):
    """ Convert the color of an object to hue

    Retrieve the first material of the first mesh of an object
    and return the hue of this material.
    """
    try:
        mesh = obj.meshes[0] # There can be more than one mesh...
    except IndexError as detail:
        # Nothing will happen if the object has no materials
        return None

    if mesh != None:
        bMat = blenderapi.materialdata(mesh.getMaterialName(0)[2:])
        return (RGBtoHue(bMat.diffuse_color))
    
    return None
