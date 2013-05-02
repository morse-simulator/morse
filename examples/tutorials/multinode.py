""" MORSE Multinode tutorial (socket, localhost)

Run as:

    multinode_server
    morse run -g 800x600 --name node1 multinode.py
    morse run -g 800x600 --name node2 multinode.py
    morse run -g 800x600 --name node3 multinode.py
    # ...
"""
from morse.builder import *

robots = {}

for idx in range(11):
    idx_robot = 'node%i' % idx
    robots[idx_robot] = ATRV('dala%i' % idx)
    kb = Keyboard('keyb')
    robots[idx_robot].append(kb)
    robots[idx_robot].translate(idx, -idx, 0)
    kb.properties(Speed=3)

env = Environment('outdoors')#, fastmode=True)
env.show_framerate(True)

env.configure_multinode(
        protocol = "socket",
        server_address = "localhost",
        server_port = "65000",
        distribution = {idx: robots[idx].name for idx in robots.keys()}
    )

env.create()
#env.set_log_level('morse.multinode.socket', 'debug')
