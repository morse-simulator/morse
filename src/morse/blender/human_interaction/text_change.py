from bge import logic

co = logic.getCurrentController()
ow = co.owner
human = logic.getSceneList()[0].objects['Hand_Grab.R']

message = co.sensors['Message']

try:
    ow['Text'] = message.bodies[0]
    if human['selected'] != '' and human['selected'] !=  'None':
        ow['Text'] = ow['Text'].replace('Pick up', '')
except IndexError:
    pass