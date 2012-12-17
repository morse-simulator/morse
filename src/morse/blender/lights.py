from morse.core import blenderapi

def change_light_energy():
    co = blenderapi.controller()
    ow = co.owner
    
    ow.energy = ow['Energy'] if ow['On'] else 0
