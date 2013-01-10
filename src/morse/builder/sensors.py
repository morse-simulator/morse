import logging; logger = logging.getLogger("morse.builder." + __name__)
import math
from morse.builder.creator import SensorCreator, bpymorse
from morse.builder.blenderobjects import *

class Accelerometer(SensorCreator):
    def __init__(self, name="Accelerometer"):
        SensorCreator.__init__(self, name, "morse/sensors/accelerometer", \
                               "AccelerometerClass", "accelerometer")
        mesh = Cube("AccelerometerCube")
        mesh.scale = (.04, .04, .02)
        mesh.color(.3, .9, .6)
        self.append(mesh)

class Battery(SensorCreator):
    def __init__(self, name="Battery"):
        SensorCreator.__init__(self, name,"morse/sensors/battery", \
                               "BatteryClass", "battery")
        mesh = Cylinder("BatteryCylinder")
        mesh.scale = (.01, .01, .04)
        mesh.color(.2, .2, .2)
        self.append(mesh)
        self.properties(DischargingRate = 0.05)

class GPS(SensorCreator):
    def __init__(self, name="GPS"):
        SensorCreator.__init__(self, name, "morse/sensors/gps", \
                               "GPSClass", "gps")
        mesh = Sphere("GPSSphere")
        mesh.scale = (.04, .04, .01)
        mesh.color(.5, .5, .5)
        self.append(mesh)

class Gyroscope(SensorCreator):
    def __init__(self, name="Gyroscope"):
        SensorCreator.__init__(self, name,"morse/sensors/gyroscope", \
                               "GyroscopeClass", "gyroscope")
        mesh = Sphere("GyroscopeSphere")
        mesh.scale = (.04, .04, .01)
        mesh.color(.8, .4, .1)
        self.append(mesh)

class IMU(SensorCreator):
    def __init__(self, name="IMU"):
        SensorCreator.__init__(self, name, "morse/sensors/imu", \
                               "ImuClass", "imu")
        mesh = Cube("IMUCube")
        mesh.scale = (.04, .04, .02)
        mesh.color(.3, .9, .6)
        self.append(mesh)

class JidoPosture(SensorCreator):
    def __init__(self, name="jido_posture"):
        SensorCreator.__init__(self, name, "morse/sensors/jido_posture", \
                               "JidoPostureClass", "jido_posture")
        self.properties(KUKAname = "KUKA_LWR", PTUname = "PTU")

class KukaPosture(SensorCreator):
    def __init__(self, name="kuka_posture"):
        SensorCreator.__init__(self, name, "morse/sensors/kuka_posture", \
                               "KukaPostureClass", "kuka_posture")
        self.properties(KUKAname = "kuka_armature")
        mesh = Cube("KukaPostureCube")
        mesh.scale = (.04, .04, .02)
        mesh.color(.6, .4, .1)
        self.append(mesh)

class Odometry(SensorCreator):
    def __init__(self, name="Odometry"):
        SensorCreator.__init__(self, name, "morse/sensors/odometry", \
                               "OdometryClass", "odometry")
        mesh = Cylinder("OdometryCylinder")
        mesh.scale = (.02, .02, .02)
        mesh.color(.5, .5, .5)
        self.append(mesh)

class Pose(SensorCreator):
    def __init__(self, name="Pose"):
        SensorCreator.__init__(self, name, "morse/sensors/pose", \
                               "PoseClass", "pose")
        mesh = Cube("PoseCube")
        mesh.scale = (.04, .04, .02)
        mesh.color(.8, .3, .1)
        self.append(mesh)

class PR2Posture(SensorCreator):
    def __init__(self, name="pr2_posture"):
        SensorCreator.__init__(self, name, "morse/sensors/pr2_posture", \
                               "PR2PostureClass", "pr2_posture")

class Proximity(SensorCreator):
    def __init__(self, name="Proximity"):
        SensorCreator.__init__(self, name, "morse/sensors/proximity", \
                               "ProximitySensorClass", "proximity")
        mesh = Cylinder("ProximityCylinder")
        mesh.scale = (.02, .02, .02)
        mesh.color(.5, .5, .5)
        self.append(mesh)
        self.properties(Range = 30.0, Track = "Robot_Tag")
        self.frequency(12)

class PTUPosture(SensorCreator):
    def __init__(self, name="ptu_posture"):
        SensorCreator.__init__(self, name, "morse/sensors/ptu_posture", \
                               "PTUPostureClass", "ptu_posture")
        self.properties(PTUname = "PTU")

class Rosace(SensorCreator):
    def __init__(self, name="rosace"):
        SensorCreator.__init__(self, name, "morse/sensors/rosace", \
                               "RosaceSensorClass", "rosace")
        mesh = Cylinder("RosaceCylinder")
        mesh.scale = (.15, .04, .04)
        mesh.color(.2, .8, .6)
        mesh.rotate(y = math.pi / 2)
        self.append(mesh)
        self.properties(Heal_range = 2.0, Abilities = "1,2")
        self.frequency(6)
        # add Radar freq: 3 Hz, prop: Injured, angle: 60.0, distance: 10.0
        bpymorse.add_sensor(type="RADAR")
        obj = bpymorse.get_context_object()
        sensor = obj.game.sensors[-1]
        sensor.angle = 60.0
        sensor.distance = 10.0
        sensor.use_pulse_true_level = True
        sensor.frequency = 20
        sensor.property = "Injured"
        # link it to the Python controller
        controller = obj.game.controllers[-1]
        controller.link(sensor = sensor)

class StereoUnit(SensorCreator):
    def __init__(self, name="stereo_unit"):
        SensorCreator.__init__(self, name, "morse/sensors/stereo_unit", \
                               "StereoUnitClass", "stereo_unit")
        mesh = Cube("StereoUnitCube")
        mesh.scale = (.025, .24, .01)
        mesh.color(.8, .8, .8)
        self.append(mesh)

class Thermometer(SensorCreator):
    def __init__(self, name="Thermometer"):
        SensorCreator.__init__(self, name, "morse/sensors/thermometer", \
                               "ThermometerClass", "thermometer")
        mesh = Cylinder("ThermometerCylinder")
        mesh.scale = (.02, .02, .04)
        mesh.color(0, .6, .5)
        self.append(mesh)

# abstract class
class LaserSensorWithArc(SensorCreator):
    def __init__(self, name, cpath, cclass, bname):
        SensorCreator.__init__(self, name, cpath, cclass, bname)
        # append sick Arc, for 'RayMat' material
        self.append_meshes(component='sick', prefix='Arc_')

    def create_laser_arc(self):
        """ Create an arc for use with the laserscanner sensor

        The arc is created using the parameters in the laserscanner Empty.
        'resolution and 'scan_window' are used to determine how many points
        will be added to the arc.
        """
        scene = bpymorse.get_context_scene()

        laserscanner_obj = self._blendobj

        material = None
        # Get the mesh and the "RayMat" material for the arc
        for mat in bpymorse.get_materials():
            if "RayMat" in mat.name:
                material = mat.material
                break

        # Delete previously created arc
        for child in laserscanner_obj.children:
            if child.name.startswith("Arc_"):
                scene.objects.unlink( child )

        # Read the parameters to create the arc
        properties = laserscanner_obj.game.properties
        resolution = properties['resolution'].value
        window = properties['scan_window'].value
        # Parameters for multi layer sensors
        try:
            layers = properties['layers'].value
            layer_separation = properties['layer_separation'].value
            layer_offset = properties['layer_offset'].value
        except KeyError as detail:
            layers = 1
            layer_separation = 0.0
            layer_offset = 0.0
        logger.debug ("Creating %d arc(s) of %.2f degrees, resolution %.2f" % \
                      (layers, window, resolution))
        mesh = bpymorse.new_mesh( "ArcMesh" )
        # Add the center vertex to the list of vertices
        verts = [ [0.0, 0.0, 0.0] ]
        faces = []
        vertex_index = 0

        # Set the vertical angle, in case of multiple layers
        if layers > 1:
            v_angle = layer_separation * (layers-1) / 2.0
        else:
            v_angle = 0.0

        # Initialise the parameters for every layer
        for layer_index in range(layers):
            start_angle = window / 2.0
            end_angle = -window / 2.0
            # Offset the consecutive layers
            if (layer_index % 2) == 0:
                start_angle += layer_offset
                end_angle += layer_offset
            logger.debug ("Arc from %.2f to %.2f" % (start_angle, end_angle))
            logger.debug ("Vertical angle: %.2f" % v_angle)
            arc_angle = start_angle

            # Create all the vertices and faces in a layer
            while arc_angle >= end_angle:
                # Compute the coordinates of the new vertex
                new_vertex = [ math.cos(math.radians(arc_angle)), \
                               math.sin(math.radians(arc_angle)), \
                               math.sin(math.radians(v_angle)) ]
                verts.append(new_vertex)
                vertex_index = vertex_index + 1
                # Add the faces after inserting the 2nd vertex
                if arc_angle < start_angle:
                    faces.append([0, vertex_index-1, vertex_index])
                # Increment the angle by the resolution
                arc_angle = arc_angle - resolution

            v_angle -= layer_separation

        mesh.from_pydata( verts, [], faces )
        mesh.update()
        # Compose the name of the arc
        arc_name = "Arc_%d" % window
        arc = bpymorse.new_object( arc_name, mesh )
        arc.data = mesh
        # Remove collision detection for the object
        arc.game.physics_type = 'NO_COLLISION'
        # Set the material of the arc
        if material: # XXX TODO check if compulsory ?
            arc.active_material = material
        # Link the new object in the scene
        scene.objects.link( arc )
        # Set the parent to be the laserscanner Empty
        arc.parent = laserscanner_obj

    def __del__(self):
        arc = [child for child in self._blendobj.children
               if child.name.startswith("Arc_")]
        if not arc:
            self.create_laser_arc()

class Hokuyo(LaserSensorWithArc):
    def __init__(self, name="Hokuyo"):
        LaserSensorWithArc.__init__(self, name, "morse/sensors/laserscanner", \
                                    "LaserScannerClass", "hokuyo")
        mesh = Cylinder("HokuyoCylinder")
        mesh.scale = (.04, .04, .08)
        mesh.color(0, 0, 0)
        self.append(mesh)
        # set components-specific properties
        self.properties(Visible_arc = False, laser_range = 30.0,
                scan_window = 270.0, resolution = 0.25)
        # set the frequency to 10 (6 scan/s for ticrate = 60Hz)
        self.frequency(10)

class Sick(LaserSensorWithArc):
    def __init__(self, name="Sick"):
        LaserSensorWithArc.__init__(self, name, "morse/sensors/laserscanner", \
                                    "LaserScannerClass", "sick")
        # set components-specific properties
        self.properties(Visible_arc = False, laser_range = 30.0,
                scan_window = 180.0, resolution = 1.0)
        # set the frequency to 10 (6 scan/s for ticrate = 60Hz)
        self.frequency(10)
        # append sick mesh, from MORSE_COMPONENTS/sensors/sick.blend
        self.append_meshes(['SickMesh'])

class SickLDMRS(LaserSensorWithArc):
    def __init__(self, name="Sick"):
        LaserSensorWithArc.__init__(self, name, "morse/sensors/laserscanner", \
                                    "LaserScannerClass", "sick-ld-mrs")
        # set components-specific properties
        self.properties(Visible_arc = False, laser_range = 30.0,
                scan_window = 100.0, resolution = 0.25, layers = 4,
                layer_separation = 0.8, layer_offset = 0.125)
        # set the frequency to 10 (6 scan/s for ticrate = 60Hz)
        self.frequency(4)
        # append sick mesh, from MORSE_COMPONENTS/sensors/sick-ld-mrs.blend
        self.append_meshes(['SickMesh'])

class Infrared(LaserSensorWithArc):
    def __init__(self, name="Infrared"):
        LaserSensorWithArc.__init__(self, name, "morse/sensors/laserscanner", \
                                    "LaserScannerClass", "infrared")
        mesh = Cube("InfraredCube")
        mesh.scale = (.02, .02, .02)
        mesh.color(.8, .8, .8)
        self.append(mesh)
        # set components-specific properties
        self.properties(Visible_arc = True, laser_range = 2.0,
                scan_window = 20.0, resolution = 1.0)
        # set the frequency to 10 (6 scan/s for ticrate = 60Hz)
        self.frequency(10)

class CameraVideo(SensorCreator):
    def __init__(self, name="CameraVideo", cclass="VideoCameraClass", \
                 cpath="morse/sensors/video_camera", bname = "video_camera"):
        SensorCreator.__init__(self, name, cpath, cclass, bname)
        camera = Camera("CameraRobot")
        self.append(camera)
        self.properties(cam_width = 256, cam_height = 256, cam_focal = 35.0, \
                        capturing = True, Vertical_Flip = True)
        # set the frequency to 3 (20ips for ticrate = 60Hz)
        self.frequency(3)
        # add toggle capture action (`Space` key)
        bpymorse.add_sensor(type="KEYBOARD")
        obj = bpymorse.get_context_object()
        sensor = obj.game.sensors[-1]
        sensor.key = 'SPACE'
        bpymorse.add_controller(type='LOGIC_AND')
        controller = obj.game.controllers[-1]
        bpymorse.add_actuator(type='PROPERTY')
        actuator = obj.game.actuators[-1]
        actuator.mode = 'TOGGLE'
        actuator.property = 'capturing'
        controller.link(sensor = sensor, actuator = actuator)
        # append CameraMesh with its textures
        imported_objects = self.append_meshes(['CameraMesh'], "video_camera")
        # TODO fix the CameraMesh location and rotation in video_camera.blend
        camera_mesh = imported_objects[0]
        camera_mesh.location = (-0.015, 0, 0)
        camera_mesh.rotation_euler = (0, 0, -math.pi)

class CameraDepth(CameraVideo):
    def __init__(self, name="CameraDepth"):
        CameraVideo.__init__(self, name, "DepthCameraClass", \
                             "morse/sensors/depth_camera", "depth_camera")
        self.properties(cam_width = 128, cam_height = 128, \
                        cam_near=1.0, cam_far=20.0, Depth=True)

class CameraSemantic(CameraVideo):
    def __init__(self, name="CameraSemantic"):
        CameraVideo.__init__(self, name, "SemanticCameraClass", \
                             "morse/sensors/semantic_camera", "semantic_camera")
        self.properties(cam_width = 512, cam_height = 512)

class Velodyne(SensorCreator):
    def __init__(self, name="Velodyne"):
        SensorCreator.__init__(self, name, "morse/sensors/velodyne", \
                               "VelodyneClass", "velodyne")
        # set components-specific properties
        self.properties(Visible_arc = True, laser_range = 50.0, \
                        scan_window = 31.500, resolution = 0.5)
        # Add Always (use_true_level) - And - Motion (rotation z: 0.017453)
        bpymorse.add_sensor() # default is Always sensor
        obj = bpymorse.get_context_object()
        sensor = obj.game.sensors[-1]
        sensor.use_pulse_true_level = True
        bpymorse.add_controller(type='LOGIC_AND')
        controller = obj.game.controllers[-1]
        # Motion (rotation z: 0.017453)
        bpymorse.add_actuator(type='MOTION')
        actuator = obj.game.actuators[-1]
        actuator.offset_rotation.z = math.radians(1)
        controller.link(sensor = sensor, actuator = actuator)
        # append velodyne mesh, from MORSE_COMPONENTS/sensors/velodyne.blend
        self.append_meshes(['VelodyneMesh', 'Arc_31'])

class Clock(SensorCreator):
    def __init__(self, name="Clock"):
        SensorCreator.__init__(self, name, "morse/core/sensor", \
                               "Sensor", "clock")
