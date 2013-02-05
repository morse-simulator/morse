from morse.builder import *

Cat = ATRV()
Cat.name = "CAT"
Cat.translate(x=-6.0, z=0.2)
#Cat.rotate(z=-1.5708)

V_W = MotionVW()
Cat.append(V_W)

Semantic_L = SemanticCamera()
Semantic_L.translate(x=0.2, y=0.3, z=0.9)
Semantic_L.name = "Camera_L"
Cat.append(Semantic_L)

Semantic_R = SemanticCamera()
Semantic_R.translate(x=0.2, y=-0.3, z=0.9)
Semantic_R.name = "Camera_R"
Cat.append(Semantic_R)

V_W.add_stream('socket')
Semantic_L.add_stream('socket')
Semantic_R.add_stream('socket')


Mouse = ATRV()
Mouse.name = "MOUSE"
Mouse.properties(Object = True, Graspable = False, Label = "MOUSE")
Mouse.translate (x=1.0, z=0.2)

Keyb = Keyboard()
Keyb.properties(Speed=3.0)
Mouse.append(Keyb)


env = Environment('land-1/trees')
env.place_camera([10.0, -10.0, 10.0])
env.aim_camera([1.0470, 0, 0.7854])
env.select_display_camera(Semantic_L)
