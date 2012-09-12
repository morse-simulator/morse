from morse.builder import *

ATRV = Robot('atrv')

ATRV_001 = Robot('atrv')
ATRV_001.translate(y=-5.0000)

GPS = Sensor('gps')
GPS.translate(x=-0.2000, z=0.9000)
ATRV.append(GPS)

GPS_001 = Sensor('gps')
GPS_001.translate(x=-0.2000, z=0.9000)
ATRV_001.append(GPS_001)

Gyroscope = Sensor('gyroscope')
Gyroscope.translate(z=0.9000)
ATRV.append(Gyroscope)

Gyroscope_001 = Sensor('gyroscope')
Gyroscope_001.translate(z=0.9000)
ATRV_001.append(Gyroscope_001)

Motion_Controller = Actuator('waypoint')
ATRV.append(Motion_Controller)
Motion_Controller.properties(Speed = 10.0000)

Motion_Controller_001 = Actuator('waypoint')
ATRV_001.append(Motion_Controller_001)
Motion_Controller_001.properties(Speed = 10.0000)

Proximity_Sensor = Sensor('proximity')
Proximity_Sensor.translate(y=0.2000, z=1.0000)
ATRV.append(Proximity_Sensor)
Proximity_Sensor.properties(Range = 30.0000)

Proximity_Sensor_001 = Sensor('proximity')
Proximity_Sensor_001.translate(y=0.2000, z=1.0000)
ATRV_001.append(Proximity_Sensor_001)
Proximity_Sensor_001.properties(Range = 30.0000)

Rosace_Sensor = Sensor('rosace')
Rosace_Sensor.translate(x=0.3000, z=0.9000)
ATRV.append(Rosace_Sensor)
Rosace_Sensor.properties(Heal_range = 2.0000)
Rosace_Sensor.properties(Abilities = '1,2')

Rosace_Sensor_001 = Sensor('rosace')
Rosace_Sensor_001.translate(x=0.3000, z=0.9000)
ATRV_001.append(Rosace_Sensor_001)
Rosace_Sensor_001.properties(Heal_range = 2.0000)
Rosace_Sensor_001.properties(Abilities = '1,2')

Thermometer = Sensor('thermometer')
Thermometer.translate(y=-0.2000, z=0.9000)
ATRV.append(Thermometer)

Thermometer_001 = Sensor('thermometer')
Thermometer_001.translate(y=-0.2000, z=0.9000)
ATRV_001.append(Thermometer_001)


GPS.configure_mw('yarp', ['morse.middleware.yarp_mw.MorseYarpClass', 'post_json_message', 'morse/middleware/yarp/json_mod'])
Motion_Controller.configure_mw('yarp', ['morse.middleware.yarp_mw.MorseYarpClass', 'read_json_waypoint', 'morse/middleware/yarp/json_mod'])
Rosace_Sensor.configure_mw('yarp', ['morse.middleware.yarp_mw.MorseYarpClass', 'post_json_message', 'morse/middleware/yarp/json_mod'])
Proximity_Sensor.configure_mw('yarp', ['morse.middleware.yarp_mw.MorseYarpClass', 'post_json_message', 'morse/middleware/yarp/json_mod'])

Rosace_Sensor_001.configure_mw('yarp', ['morse.middleware.yarp_mw.MorseYarpClass', 'post_json_message', 'morse/middleware/yarp/json_mod'])
GPS_001.configure_mw('yarp', ['morse.middleware.yarp_mw.MorseYarpClass', 'post_json_message', 'morse/middleware/yarp/json_mod'])
Proximity_Sensor_001.configure_mw('yarp', ['morse.middleware.yarp_mw.MorseYarpClass', 'post_json_message', 'morse/middleware/yarp/json_mod'])
Motion_Controller_001.configure_mw('yarp', ['morse.middleware.yarp_mw.MorseYarpClass', 'read_json_waypoint', 'morse/middleware/yarp/json_mod'])

Motion_Controller.configure_service('yarp_json')
Rosace_Sensor_001.configure_service('yarp_json')
Motion_Controller_001.configure_service('yarp_json')
Rosace_Sensor.configure_service('yarp_json')

# Victims
victim1 = Robot('victim')
victim1.translate(x=4.0)
victim1.properties(Requirements = "1,2,3")

victim2 = Robot('victim')
victim2.translate(x=8.0, y=5.0)
victim2.properties(Requirements = "1")

env = Environment('land-1/rosace_1')
env.place_camera([9, -9, 9])
