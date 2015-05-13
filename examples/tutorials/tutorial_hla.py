from morse.builder import *

foo = Morsy()
foo.set_color((1.0, 0.0, 0.0))
foo.scale = [10.0, 10.0, 1]

teleport = Teleport()
teleport.add_stream('hla', 'morse.middleware.hla.certi_test_input.CertiTestInput')
foo.append(teleport)

pose = Pose()
foo.append(pose)
pose.add_stream('socket')

bar = Morsy()
bar.scale = [10.0, 10.0, 1]
bar.translate(x = 12, y = 12)

kb = Keyboard()
bar.append(kb)
kb.properties(Speed = 10.0)

pose2 = Pose()
bar.append(pose2)
pose2.add_stream('hla', 'morse.middleware.hla.certi_test_output.CertiTestOutput')

env = Environment('empty')

env.configure_stream_manager(
        'hla', 
        fom = 'Test.fed', name = 'Morse', federation = 'Test', sync_point = 'Init', time_sync = True, timestep = 1.0)

ground = bpymorse.get_object('Ground')
ground.scale = [255.0, 55.0, 0.0065]
ground.location = [250.0, 50.0, -0.06]
env.set_camera_clip(0.1, 1000)
env.set_camera_location([250, 50, 350])
env.set_camera_rotation([0.0, 0.0, 0.0])
env.set_camera_speed(10.0)
