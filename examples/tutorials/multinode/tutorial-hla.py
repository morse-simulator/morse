from morse.builder.morsebuilder import *

from dala_simple import equipped_robot

dala1 = equipped_robot(mw='yarp')
dala2 = equipped_robot(mw='yarp')
dala2.translate(5, -3, 0)

env = Environment('laas/grande_salle')
env.show_framerate(True)
env.show_physics(False)

env.configure_multinode(protocol="hla", server_port=60400, 
    distribution={
        "nodeA": [dala1.name],
        "nodeB": [dala2.name],
    })

env.aim_camera([1.3300, 0, 0.7854])
env.place_camera([10.0, -10.0, 3.0])

env.create()
