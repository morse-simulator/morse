from morse.builder import *

# PR1 robot
pr2 = BasePR2()
pr2.add_interface('socket')

env = Environment('indoors-1/indoor-1')
