from morse.core import blenderapi

def set_body_position(co):
    """
    During grabbing the head moves. The 'Head_Empty' needs to follow.
    """
    ow = co.owner

    # get the suffix of the human to reference the right objects
    suffix = ow.name[-4:] if ow.name[-4] == "." else ""

    head = blenderapi.scene().objects['Head']
    human = blenderapi.scene().objects[ow.parent["human_name"]]



    if human['Manipulate']:
        ow.worldPosition = head.worldPosition
