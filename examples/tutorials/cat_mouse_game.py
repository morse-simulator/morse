from morse.builder import *

Cat = Robot('atrv')
Cat.name = "CAT"
Cat.translate(x=-6.0, z=0.2)
#Cat.rotate(z=-1.5708)

V_W = Actuator('v_omega')
Cat.append(V_W)

Semantic_L = Sensor('semantic_camera')
Semantic_L.translate(x=0.2, y=0.3, z=0.9)
Semantic_L.name = "Camera_L"
Cat.append(Semantic_L)

Semantic_R = Sensor('semantic_camera')
Semantic_R.translate(x=0.2, y=-0.3, z=0.9)
Semantic_R.name = "Camera_R"
Cat.append(Semantic_R)

V_W.configure_mw('socket')
Semantic_L.configure_mw('socket')
Semantic_R.configure_mw('socket')


Mouse = Robot('atrv')
Mouse.name = "MOUSE"
Mouse.properties(Object = True, Graspable = False, Label = "MOUSE")
Mouse.translate (x=1.0, z=0.2)

Keyb = Actuator('keyboard')
Keyb.properties(Speed=3.0)
Mouse.append(Keyb)


env = Environment('land-1/trees')
env.place_camera([10.0, -10.0, 10.0])
env.aim_camera([1.0470, 0, 0.7854])
env.select_display_camera(Semantic_L)
