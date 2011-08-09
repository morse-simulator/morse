import logging; logger = logging.getLogger("morse." + __name__)
def reset_rotation (contr):
    """ Cancell the global rotation of the object

    This will make it remain still, as a "billboard",
    with respect to the user controlled camera
    """
    obj = contr.owner
    obj.worldOrientation = [0.0, 0.0, 0.0]


def display (contr):
    """ Toggle showing or hiding of component """
    if contr.sensors[0].positive:
        obj = contr.owner
        obj['Display'] = not obj['Display']
        obj.setVisible(obj['Display'])
