from morse.builder import *

r = ATRV()
v = VideoCamera()
v.translate(z=1)
r.append(v)
v.add_stream('socket')
k = Keyboard()
r.append(k)

e = Environment('outdoors')
