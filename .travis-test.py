from morse.builder import *

atrv = Robot('atrv')

motion = Actuator('v_omega')
motion.name = 'Motion'
motion.translate(z = 0.3)
atrv.append(motion)

pose = Sensor('pose')
pose.name = 'Odometry'
pose.translate(x = -0.25, z = 0.83)
atrv.append(pose)

motion.configure_mw('ros')
pose.configure_mw('ros')

env = Environment('indoors-1/indoor-1')
env.aim_camera([1.0470, 0, 0.7854])

# save the scene
bpy.ops.wm.save_mainfile(filepath = "saved.blend")

print("test successfully passed!")

