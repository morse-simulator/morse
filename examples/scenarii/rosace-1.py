from morse.builder import *

atrv00 = ATRV()

atrv01 = ATRV()
atrv01.translate(y=-5.0000)

gps00 = GPS()
gps00.translate(x=-0.2000, z=0.9000)
atrv00.append(gps00)

gps01 = GPS()
gps01.translate(x=-0.2000, z=0.9000)
atrv01.append(gps01)

gyroscope00 = Gyroscope()
gyroscope00.translate(z=0.9000)
atrv00.append(gyroscope00)

gyroscope01 = Gyroscope()
gyroscope01.translate(z=0.9000)
atrv01.append(gyroscope01)

waipont00 = Waypoint()
atrv00.append(waipont00)
waipont00.properties(Speed = 10.0000)

waipont01 = Waypoint()
atrv01.append(waipont01)
waipont01.properties(Speed = 10.0000)

proximity00 = Proximity()
proximity00.translate(y=0.2000, z=1.0000)
atrv00.append(proximity00)
proximity00.properties(Range = 30.0000)

proximity01 = Proximity()
proximity01.translate(y=0.2000, z=1.0000)
atrv01.append(proximity01)
proximity01.properties(Range = 30.0000)

rosace00 = SearchAndRescue()
rosace00.translate(x=0.3000, z=0.9000)
atrv00.append(rosace00)
rosace00.properties(Heal_range = 2.0000)
rosace00.properties(Abilities = '1,2')

rosace01 = SearchAndRescue()
rosace01.translate(x=0.3000, z=0.9000)
atrv01.append(rosace01)
rosace01.properties(Heal_range = 2.0000)
rosace01.properties(Abilities = '1,2')

thermometer00 = Thermometer()
thermometer00.translate(y=-0.2000, z=0.9000)
atrv00.append(thermometer00)

thermometer01 = Thermometer()
thermometer01.translate(y=-0.2000, z=0.9000)
atrv01.append(thermometer01)


gps00.add_stream('yarp', 'post_json_message', 'morse/middleware/yarp/json_mod')
waipont00.add_stream('yarp', 'read_json_waypoint', 'morse/middleware/yarp/json_mod')
rosace00.add_stream('yarp', 'post_json_message', 'morse/middleware/yarp/json_mod')
proximity00.add_stream('yarp', 'post_json_message', 'morse/middleware/yarp/json_mod')

gps01.add_stream('yarp', 'post_json_message', 'morse/middleware/yarp/json_mod')
waipont01.add_stream('yarp', 'read_json_waypoint', 'morse/middleware/yarp/json_mod')
rosace01.add_stream('yarp', 'post_json_message', 'morse/middleware/yarp/json_mod')
proximity01.add_stream('yarp', 'post_json_message', 'morse/middleware/yarp/json_mod')

waipont00.add_service('yarp_json')
rosace00.add_service('yarp_json')

waipont01.add_service('yarp_json')
rosace01.add_service('yarp_json')

# Victims
victim1 = Victim()
victim1.translate(x=4.0)
victim1.properties(Requirements = "1,2,3")

victim2 = Victim()
victim2.translate(x=8.0, y=5.0)
victim2.properties(Requirements = "1")

env = Environment('land-1/rosace_1')
env.place_camera([9, -9, 9])
