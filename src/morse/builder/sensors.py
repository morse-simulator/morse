import bpy
import morse.builder.creator
import morse.builder.blenderobjects

class GPS(morse.builder.creator.SensorCreator):
    def __init__(self, name="GPS"):
        morse.builder.creator.SensorCreator.__init__(self, name, 
            "morse/sensors/gps", "GPSClass", "gps")
        mesh = morse.builder.blenderobjects.Cylinder("GPSCylinder")
        mesh.scale = (.1, .1, .05)
        self.append(mesh)

class Odometry(morse.builder.creator.SensorCreator):
    def __init__(self, name="Odometry"):
        morse.builder.creator.SensorCreator.__init__(self, name, 
            "morse/sensors/odometry", "OdometryClass", "odometry")
        mesh = morse.builder.blenderobjects.Cube("OdometryCube")
        mesh.scale = (.1, .1, .05)
        self.append(mesh)

class Pose(morse.builder.creator.SensorCreator):
    def __init__(self, name="Pose_sensor"):
        morse.builder.creator.SensorCreator.__init__(self, name, 
            "morse/sensors/pose", "PoseClass", "pose")
        mesh = morse.builder.blenderobjects.Cylinder("PoseCylinder")
        mesh.scale = (.1, .1, .2)
        self.append(mesh)

class Proximity(morse.builder.creator.SensorCreator):
    def __init__(self, name="Proximity"):
        morse.builder.creator.SensorCreator.__init__(self, name, 
            "morse/sensors/proximity", "ProximitySensorClass", "proximity")
        mesh = morse.builder.blenderobjects.Cylinder("ProximityCylinder")
        mesh.scale = (.1, .1, .2)
        self.append(mesh)
        self.properties(Range = 30.0)

##
# Following is experimental (!)
##

class Sick(morse.builder.creator.SensorCreator):
    def __init__(self, name="Sick"):
        morse.builder.creator.SensorCreator.__init__(self, name, 
            "morse/sensors/sick", "SICKClass", "sick")
        # set components-specific properties
        self.properties(Visible_arc = True, laser_range = 30.0, 
                scan_window = 180.0, resolution = 1.0)
        # set the frequency to 10 (6 scan/s for ticrate = 60Hz)
        self.frequency(10)
        # append sick mesh, from MORSE_COMPONENTS/sensors/sick.blend
        self.append_meshes(['Arc_180', 'Sick_Model'])

class Infrared(morse.builder.creator.SensorCreator):
    def __init__(self, name="Infrared"):
        morse.builder.creator.SensorCreator.__init__(self, name, 
            "morse/sensors/sick", "SICKClass", "infrared")
        # set components-specific properties
        self.properties(Visible_arc = True, laser_range = 2.0, 
                scan_window = 20.0, resolution = 1.0)
        # set the frequency to 10 (6 scan/s for ticrate = 60Hz)
        self.frequency(10)
        # append sick mesh, from MORSE_COMPONENTS/sensors/infrared.blend
        self.append_meshes(['Arc_20', 'Cube'])

def get_properties_str(name):
    """ Returns the Game properties of the Blender object represented by the name
    ie. get_properties_str('Sick')
    # laser_range = 30.0, Component_Tag = True, scan_window = 180.0, Visible_arc = True, Path = 'morse/sensors/sick', resolution = 0.25, Class = 'SICKClass'
    """
    o = bpy.data.objects[name]
    d = get_properties(o)
    return ", ".join(["%s = %s"%(k,d[k]) for k in d])

def get_properties(o):
    d = {}
    p = o.game.properties
    for k in p.keys():
        d[k] = p[k].value
        if p[k].type == 'TIMER':
            d[k] = "timer(%f)"%d[k]
        elif type(d[k]) is str:
            d[k] = "'%s'"%d[k]
    return d

class Camera(morse.builder.creator.SensorCreator):
    def __init__(self, name="CameraMain"):
        morse.builder.creator.SensorCreator.__init__(self, name, 
            "morse/sensors/video_camera", "VideoCameraClass", "video_camera")
        camera = morse.builder.blenderobjects.Camera("CameraRobot")
        self.append(camera)
        self.properties(cam_width = 256, cam_height = 256, cam_focal = 35.0, 
                capturing = True, Vertical_Flip = True)
        # set the frequency to 3 (20ips for ticrate = 60Hz)
        self.frequency(3)
        # add toggle capture action (`Space` key)
        bpy.ops.object.select_all(action = 'DESELECT')
        bpy.ops.object.select_name(name = self._blendobj.name)
        bpy.ops.logic.sensor_add(type="KEYBOARD")
        sensor = self._blendobj.game.sensors[-1]
        sensor.key = 'SPACE'
        bpy.ops.logic.controller_add(type='LOGIC_AND')
        controller = self._blendobj.game.controllers[-1]
        bpy.ops.logic.actuator_add(type='PROPERTY')
        actuator = self._blendobj.game.actuators[-1]
        actuator.mode = 'TOGGLE'
        actuator.property = 'capturing'
        controller.link(sensor = sensor, actuator = actuator)
        # append CameraMesh with its textures
        self.append_meshes(['CameraMesh'])

class Battery(morse.builder.creator.SensorCreator):
    def __init__(self, name="Battery"):
        morse.builder.creator.SensorCreator.__init__(self, name, 
            "morse/sensors/battery", "BatteryClass", "battery")
        self.properties(DischargingRate = 1.0)

class Clock(morse.builder.creator.SensorCreator):
    def __init__(self, name="Clock"):
        morse.builder.creator.SensorCreator.__init__(self, name, 
            "morse/core/sensor", "MorseSensorClass", "clock")

