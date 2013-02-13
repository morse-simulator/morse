from morse.builder import *

robot = FakeRobot()
# Kuka arm
kuka = KukaLWR()
kuka.add_service('socket')
robot.append(kuka)

env = Environment('indoors-1/indoor-1')
