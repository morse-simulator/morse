from morse.builder import *

r = ATRV()
v = VideoCamera()
v.properties(cam_width=800, cam_height=600)
v.translate(z=1)
r.append(v)
v.add_stream('socket')
k = Keyboard()
r.append(k)

e = Environment('outdoors')
