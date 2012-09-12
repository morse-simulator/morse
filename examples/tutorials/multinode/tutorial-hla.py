# Add the path of this file to PYTHONPATH
import os, sys, inspect
# cmd_folder = os.path.dirname(os.path.abspath(__file__)) # DO NOT USE __file__ !!!
# __file__ fails if script is called in different ways on Windows
# __file__ fails if someone does os.chdir() before
# sys.argv[0] also fails because it doesn't not always contains the path
cmd_folder = os.path.abspath(os.path.split(inspect.getfile( inspect.currentframe() ))[0])
if cmd_folder not in sys.path:
        sys.path.insert(0, cmd_folder)


from morse.builder import *

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
