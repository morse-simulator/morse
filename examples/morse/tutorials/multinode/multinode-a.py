from morse.builder.morsebuilder import *

import dala_simple

dala1 = dala_simple.equipped_robot()
dala2 = dala_simple.equipped_robot()
dala2.translate(5, -3, 0)
dala2.make_external()

env = Environment('laas/grande_salle')
env.show_framerate(True)
env.show_physics(False)
env.configure_node(protocol="socket", node_name="NODE A")
#env.configure_node(protocol="socket", node_name="NODE A", server_address="140.93.0.93", server_port="65000")
env.aim_camera([1.3300, 0, 0.7854])
env.place_camera([10.0, -10.0, 3.0])
env.create()
