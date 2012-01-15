from bge import logic

def change_light_energy():
    co = logic.getCurrentController()
    ow = co.owner
    
    ow.energy = ow['Energy'] if ow['On'] else 0
