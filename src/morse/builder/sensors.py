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

class Camera(morse.builder.creator.SensorCreator):
    def __init__(self, name="CameraMain"):
        morse.builder.creator.SensorCreator.__init__(self, name, 
            "morse/sensors/video_camera", "VideoCameraClass", "camera")
        camera = morse.builder.blenderobjects.Camera("CameraRobot")
        self.append(camera)
        self.properties(cam_width = 512, cam_height = 512, cam_focal = 35.0, 
                flip = True, capturing = False)
        # add toggle capture action (`Space` key)
        bpy.ops.object.select_all(action = 'DESELECT')
        bpy.ops.object.select_name(name = self._blendobj.name)
        bpy.ops.logic.sensor_add(type="KEYBOARD")
        sensor = self._blendobj.game.sensors.keys()[-1]
        self._blendobj.game.sensors[sensor].key = 'SPACE'
        bpy.ops.logic.controller_add(type='LOGIC_AND')
        controller = self._blendobj.game.controllers.keys()[-1]
        bpy.ops.logic.actuator_add(type='PROPERTY')
        actuator = self._blendobj.game.actuators.keys()[-1]
        self._blendobj.game.actuators[actuator].mode = 'TOGGLE'
        self._blendobj.game.actuators[actuator].property = 'capturing'
        self._blendobj.game.controllers[controller].link( 
                sensor = self._blendobj.game.sensors[sensor], 
                actuator = self._blendobj.game.actuators[actuator])
        # cf. morse.sensors.camera.VideoCameraClass._setup_video_texture
        mesh = morse.builder.blenderobjects.Cube("CameraCube")
        mesh.scale = (.1, .1, .05)
        self.append(mesh)
        # XXX add "MAScreenMat" to "CameraCube" (!) or "ScreenMat" (?)
        bpy.ops.object.select_all(action = 'DESELECT')
        bpy.ops.object.select_name(name = "CameraCube")
        bpy.ops.material.new()
        # (?) bpy.ops.object.material_slot_add()
        # (?) bpy.data.materials[-1].name = "MAScreenMat"

class Battery(morse.builder.creator.SensorCreator):
    def __init__(self, name="Battery"):
        morse.builder.creator.SensorCreator.__init__(self, name, 
            "morse/sensors/battery", "BatteryClass", "battery")
        self.properties(DischargingRate = 1.0)

class Infrared(morse.builder.creator.SensorCreator):
    def __init__(self, name="Infrared"):
        morse.builder.creator.SensorCreator.__init__(self, name, 
            "morse/sensors/infrared", "InfraredClass", "infrared")
        self.properties(ir_range = 1.5)
        mesh = morse.builder.blenderobjects.Cube("InfraredCube")
        mesh.scale = (.1, .1, .05)
        self.append(mesh)
        bpy.ops.logic.sensor_remove(sensor="Always")
        bpy.ops.logic.sensor_add(type = 'RADAR') # add Radar sensor
        sensor = self._blendobj.game.sensors.keys()[-1]
        self._blendobj.game.sensors[sensor].use_pulse_true_level = True
        self._blendobj.game.sensors[sensor].angle = 5
        self._blendobj.game.sensors[sensor].distance = 20
        self._blendobj.game.sensors[sensor].axis = 'XAXIS' # default
        controller = self._blendobj.game.controllers.keys()[-1]
        self._blendobj.game.controllers[controller].link( sensor = 
                self._blendobj.game.sensors[sensor] )

class Clock(morse.builder.creator.SensorCreator):
    def __init__(self, name="Clock"):
        morse.builder.creator.SensorCreator.__init__(self, name, 
            "morse/sensors/clock", "ClockClass", "clock")

