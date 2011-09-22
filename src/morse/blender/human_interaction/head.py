from bge import logic

def set_body_position(co):
    ow = co.owner

    head = logic.getCurrentScene().objects['Head']
    human = logic.getCurrentScene().objects['POS_EMPTY']


    if human['Manipulate']:
        ow.worldPosition = head.worldPosition
