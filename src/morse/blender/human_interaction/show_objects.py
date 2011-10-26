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

            # iterate over all verticies to get the highest
            m_i = 0

            try:
                mesh = obj.meshes[m_i]
            except IndexError:
                # It seems our object has no mesh attached. It's
                # probably an empty. In that case, the mesh is
                # usually set as the *parent* of the empty.
                try:
                    mesh = obj.parent.meshes[m_i]
                except IndexError:
                    # The parent has no meshes either? hum...
                    # Then give up...
                    logger.warning("I was unable to place the %s label, since I couldn't " +\
                                   "find any mesh attached to this object or its parent!" % obj)
                    continue

            # There can be more than one mesh
            # see Blender API for more information
            z = 0
            while mesh != None:
                for mat in range(0, mesh.numMaterials):
                  for vert_id in range(mesh.getVertexArrayLength(mat)):
                    vertex = mesh.getVertex(mat, vert_id)
                    if vertex.z>z:
                        z = vertex.z
                m_i += 1
                try:
                    mesh = obj.meshes[m_i]
                except IndexError:
                    mesh = None

            textObj.applyMovement([0.0, 0.0, z*1.2])
            # set the text over the highest vertex


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

