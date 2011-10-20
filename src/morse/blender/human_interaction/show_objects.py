from bge import logic, events

scene = logic.getCurrentScene()
co = logic.getCurrentController()
keyboard = co.sensors['All_Keys']

def show(contr):
    """
    Add a text over all interactable Objects in the scene
    """

    for obj in scene.objects:
        if 'Object' in obj:
            textObj = scene.addObject('Text_proxy', obj, 0)
            # create a new instance of a text object
            if 'Description' in obj:
                textObj['Text'] = obj['Description']
            else:
                textObj['Text'] = obj['Object']
            textObj['tmp'] = 'tmp'
            # Property to identify all added text objects
            textObj.setParent(obj, False, True)
            
            # iterate over all verticies to get the highest
            m_i = 0
            mesh = obj.meshes[m_i]
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
    Delete all descriptions (Property 'tmp')
    """
    for obj in scene.objects:
        if 'tmp' in obj:
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
                
