import logging; logger = logging.getLogger("morse." + __name__)

from bge import logic, events

from morse.helpers import passive_objects

co = logic.getCurrentController()
keyboard = co.sensors['All_Keys']

def show(contr):
    """
    Add a text over all interactable Objects in the scene
    """

    scene = logic.getCurrentScene()

    for obj in passive_objects.active_objects():
            textObj = scene.addObject('Text_proxy', obj, 0)

            # create a new instance of a text object
            textObj['Text'] = passive_objects.label(obj)

            # Property to identify all added text objects
            textObj['_tooltip'] = True
            textObj.setParent(obj, False, True)

            # There can be more than one mesh
            # see Blender API for more information
            if obj.meshes:
                meshes = obj.meshes
            else:
                # It seems our object has no mesh attached. It's
                # probably an empty. In that case, check the children's
                # meshes
                if obj.children:
                    meshes = []
                    for child in obj.children:
                        meshes += child.meshes
                else:
                    # No children? hum...
                    # Then give up...
                    logger.warning("I was unable to place the %s label, since I couldn't " +\
                                   "find any mesh attached to this object or its children!" % obj)
                    continue

            z = 0
            # iterate over all verticies to get the highest
            for mesh in meshes:
                for mat in range(0, mesh.numMaterials):
                  for vert_id in range(mesh.getVertexArrayLength(mat)):
                    vertex = mesh.getVertex(mat, vert_id)
                    if vertex.z>z:
                        z = vertex.z

            # set the text over the highest vertex
            textObj.applyMovement([0.0, 0.0, z*1.2])


def hide(contr):
    """
    Delete all descriptions (Property '_tooltip')
    """
    for obj in logic.getCurrentScene().objects:
        if '_tooltip' in obj:
            obj.endObject()

def test(contr):
    """
    Test whether to show or hide the text
    """
    keylist = keyboard.events

    for key in keylist:
        if key[0] == events.LEFTALTKEY:
            if key[1] == logic.KX_INPUT_JUST_ACTIVATED:
                show(contr)
                #show text over all objects
            elif key[1] == logic.KX_INPUT_JUST_RELEASED:
                hide(contr)
                #hide text of all objects

