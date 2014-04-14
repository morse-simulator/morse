import logging; logger = logging.getLogger("morse.builder." + __name__)
import math
from morse.builder.creator import SensorCreator, bpymorse
from morse.builder.blenderobjects import *

class Accelerometer(SensorCreator):
    _classpath = "morse.sensors.accelerometer.Accelerometer"

    def __init__(self, name=None):
        SensorCreator.__init__(self, name)
        mesh = Cube("AccelerometerCube")
        mesh.scale = (.04, .04, .02)
        mesh.color(.3, .9, .6)
        self.append(mesh)

class ArmaturePose(SensorCreator):
    _classpath = "morse.sensors.armature_pose.ArmaturePose"

    def __init__(self, name=None):
        SensorCreator.__init__(self, name)

class Battery(SensorCreator):
    _classpath = "morse.sensors.battery.Battery"

    def __init__(self, name=None):
        SensorCreator.__init__(self, name)
        mesh = Cylinder("BatteryCylinder")
        mesh.scale = (.01, .01, .04)
        mesh.color(.2, .2, .2)
        self.append(mesh)
        self.properties(DischargingRate = 0.05)

class CompoundSensor(SensorCreator):
    _classpath = "morse.sensors.compound.CompoundSensor"

    def __init__(self, sensors, name=None):
        SensorCreator.__init__(self, name)

        self.sensors = sensors

    def after_renaming(self):
        # Ensures this is called only when all components have been (automatically) renamed.
        self.properties(sensors = ",".join([str(s) for s in self.sensors]))

class GPS(SensorCreator):
    _classpath  = "morse.sensors.gps.GPS"

    def __init__(self, name=None):
        SensorCreator.__init__(self, name)
        mesh = Sphere("GPSSphere")
        mesh.scale = (.04, .04, .01)
        mesh.color(.5, .5, .5)
        self.append(mesh)

class Gyroscope(SensorCreator):
    _classpath = "morse.sensors.gyroscope.Gyroscope"

    def __init__(self, name=None):
        SensorCreator.__init__(self, name)
        mesh = Sphere("GyroscopeSphere")
        mesh.scale = (.04, .04, .01)
        mesh.color(.8, .4, .1)
        self.append(mesh)

class IMU(SensorCreator): 
    _classpath = "morse.sensors.imu.IMU"

    def __init__(self, name=None):
        SensorCreator.__init__(self, name)
        mesh = Cube("IMUCube")
        mesh.scale = (.04, .04, .02)
        mesh.color(.3, .9, .6)
        self.append(mesh)

class JidoPosture(SensorCreator):
    _classpath = "morse.sensors.jido_posture.JidoPosture"

    def __init__(self, name=None):
        SensorCreator.__init__(self, name)
        self.properties(KUKAname = "KUKA_LWR", PTUname = "PTU")

class Odometry(SensorCreator):
    _classpath = "morse.sensors.odometry.Odometry"

    def __init__(self, name=None):
        SensorCreator.__init__(self, name)
        mesh = Cylinder("OdometryCylinder")
        mesh.scale = (.02, .02, .02)
        mesh.color(.5, .5, .5)
        self.append(mesh)

class Pose(SensorCreator):
    _classpath = "morse.sensors.pose.Pose"

    def __init__(self, name=None):
        SensorCreator.__init__(self, name)
        mesh = Cube("PoseCube")
        mesh.scale = (.04, .04, .02)
        mesh.color(.8, .3, .1)
        self.append(mesh)

class Proximity(SensorCreator):
    _classpath = "morse.sensors.proximity.Proximity"

    def __init__(self, name=None):
        SensorCreator.__init__(self, name)
        mesh = Cylinder("ProximityCylinder")
        mesh.scale = (.02, .02, .02)
        mesh.color(.5, .5, .5)
        self.append(mesh)
        self.properties(Range = 30.0, Track = "Robot_Tag")
        self.frequency(12)

class PTUPosture(SensorCreator):
    _classpath = "morse.sensors.ptu_posture.PTUPosture"

    def __init__(self, name=None):
        SensorCreator.__init__(self, name)

class SearchAndRescue(SensorCreator):
    _classpath = "morse.sensors.search_and_rescue.SearchAndRescue"

    def __init__(self, name=None):
        SensorCreator.__init__(self, name)
        mesh = Cylinder("SearchAndRescueCylinder")
        mesh.scale = (.15, .04, .04)
        mesh.color(.2, .8, .6)
        mesh.rotate(y = math.pi / 2)
        self.append(mesh)
        self.frequency(6)
        self.properties(Heal_range = 2.0, Abilities = "1,2")
        # add Radar freq: 3 Hz, prop: Injured, angle: 60.0, distance: 10.0
        bpymorse.add_sensor(type="RADAR", name="Radar")
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

    def properties(self, **kwargs):
        self.select()
        # We may be use it before the definition of radar
        # But angle and distance can only be defined by user, in case
        # where we are sure that Radar is well defined
        try:
            radar = self._bpy_object.game.sensors["Radar"]
            if 'Angle' in kwargs:
                radar.angle = kwargs['Angle']
            if 'Distance' in kwargs:
                radar.distance = kwargs['Distance']
            if 'Freq' in kwargs:
                radar.freq = kwargs['Freq']
        except KeyError:
            pass
        SensorCreator.properties(self, **kwargs)

class StereoUnit(SensorCreator):
    _classpath = "morse.sensors.stereo_unit.StereoUnit"

    def __init__(self, name=None):
        SensorCreator.__init__(self, name)
        mesh = Cube("StereoUnitCube")
        mesh.scale = (.025, .24, .01)
        mesh.color(.8, .8, .8)
        self.append(mesh)

class Thermometer(SensorCreator):
    _classpath = "morse.sensors.thermometer.Thermometer"

    def __init__(self, name=None):
        SensorCreator.__init__(self, name)
        mesh = Cylinder("ThermometerCylinder")
        mesh.scale = (.02, .02, .04)
        mesh.color(0, .6, .5)
        self.append(mesh)

# abstract class
class LaserSensorWithArc(SensorCreator):
    _classpath  = "morse.sensors.laserscanner.LaserScanner"

    def get_ray_material(self):
        if 'RayMat' in bpymorse.get_materials():
            return bpymorse.get_material('RayMat')

        ray_material = bpymorse.create_new_material()
        ray_material.diffuse_color = (.9, .05, .2)
        ray_material.use_shadeless = True
        ray_material.use_raytrace = False # ? is it needed ?
        ray_material.use_cast_buffer_shadows = False # ? is it needed ?
        ray_material.use_cast_approximate = False # ? is it needed ?
        ray_material.use_transparency = True
        ray_material.transparency_method = 'Z_TRANSPARENCY'
        ray_material.alpha = 0.3
        ray_material.name = 'RayMat'
        # link material to object as: Arc_XXX.active_material = ray_material
        return ray_material

    def create_laser_arc(self):
        """ Create an arc for use with the laserscanner sensor

        The arc is created using the parameters in the laserscanner Empty.
        'resolution and 'scan_window' are used to determine how many points
        will be added to the arc.
        """
        scene = bpymorse.get_context_scene()
        laserscanner_obj = self._bpy_object

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
            start_angle = -window / 2.0
            end_angle = window / 2.0
            # Offset the consecutive layers
            if (layer_index % 2) == 0:
                start_angle += layer_offset
                end_angle += layer_offset
            logger.debug ("Arc from %.2f to %.2f" % (start_angle, end_angle))
            logger.debug ("Vertical angle: %.2f" % v_angle)
            arc_angle = start_angle

            # Create all the vertices and faces in a layer
            while arc_angle <= end_angle:
                # Compute the coordinates of the new vertex
                new_vertex = [ math.cos(math.radians(arc_angle)),
                               math.sin(math.radians(arc_angle)),
                               math.sin(math.radians(v_angle)) ]
                verts.append(new_vertex)
                vertex_index += 1
                # Add the faces after inserting the 2nd vertex
                if arc_angle > start_angle:
                    faces.append([0, vertex_index-1, vertex_index])
                # Increment the angle by the resolution
                arc_angle = arc_angle + resolution

            v_angle -= layer_separation

        mesh.from_pydata( verts, [], faces )
        mesh.update()
        # Compose the name of the arc
        arc_name = "Arc_%d" % window
        arc = bpymorse.new_object( arc_name, mesh )
        arc.data = mesh
        # Remove collision detection for the object
        arc.game.physics_type = 'NO_COLLISION'
        arc.hide_render = True
        # Link the new object in the scene
        scene.objects.link( arc )
        # Set the parent to be the laserscanner Empty
        arc.parent = laserscanner_obj
        # Set the material of the arc
        arc.active_material = self.get_ray_material()
        return arc

    def after_renaming(self):
        arc = [child for child in self._bpy_object.children
               if child.name.startswith("Arc_")]
        if not arc:
            self.create_laser_arc()

class Hokuyo(LaserSensorWithArc):
    _blendname = "sick"

    def __init__(self, name=None):
        LaserSensorWithArc.__init__(self, name)
        mesh = Cylinder("HokuyoCylinder")
        mesh.scale = (.04, .04, .08)
        mesh.color(0, 0, 0)
        self.append(mesh)
        # set components-specific properties
        self.properties(Visible_arc = False, laser_range = 30.0,
                scan_window = 270.0, resolution = 0.25)
        # set the frequency to 10 Hz
        self.frequency(10)

class Sick(LaserSensorWithArc):
    _blendname = "sick"

    def __init__(self, name=None):
        LaserSensorWithArc.__init__(self, name)
        # set components-specific properties
        self.properties(Visible_arc = False, laser_range = 30.0,
                scan_window = 180.0, resolution = 1.0)
        # set the frequency to 10 Hz
        self.frequency(10)
        # append sick mesh, from MORSE_COMPONENTS/sensors/sick.blend
        self.append_meshes(['SickMesh'])

class SickLDMRS(LaserSensorWithArc):
    _blendname = "sick"

    def __init__(self, name=None):
        LaserSensorWithArc.__init__(self, name)
        # set components-specific properties
        self.properties(Visible_arc = False, laser_range = 30.0,
                scan_window = 100.0, resolution = 0.25, layers = 4,
                layer_separation = 0.8, layer_offset = 0.125)
        mesh = Cube("SickMesh")
        mesh.scale = (.05, .0825, .044)
        mesh.color(1., 1., .9)
        self.append(mesh)
        # set the frequency to 4 Hz
        self.frequency(4)

class Infrared(LaserSensorWithArc):
    def __init__(self, name=None):
        LaserSensorWithArc.__init__(self, name)
        mesh = Cube("InfraredCube")
        mesh.scale = (.02, .02, .02)
        mesh.color(.8, .8, .8)
        self.append(mesh)
        # set components-specific properties
        self.properties(Visible_arc = True, laser_range = 2.0,
                scan_window = 20.0, resolution = 1.0)
        # set the frequency to 10 Hz
        self.frequency(10)
        # create default Arc_
        self.create_laser_arc()

class Velocity(SensorCreator):
    _classpath = "morse.sensors.velocity.Velocity"

    def __init__(self, name=None):
        SensorCreator.__init__(self, name)
        mesh = Sphere("VelocitySphere")
        mesh.scale = (.04, .04, .01)
        mesh.color(.5, .5, .5)
        self.append(mesh)

class VideoCamera(SensorCreator):
    _classpath = "morse.sensors.video_camera.VideoCamera"
    _blendname = "camera"
    def __init__(self, name=None):
        SensorCreator.__init__(self, name)
        self.camera = Camera("CameraRobot")
        self.camera.name = "CameraRobot"
        self.append(self.camera)
        self.properties(cam_width = 256, cam_height = 256, cam_focal = 35.0,
                        capturing = True, Vertical_Flip = True)
        # set the frequency to 20 Hz
        self.frequency(20)
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
        # looking in +X
        SensorCreator.rotate(self, x=math.pi/2, z=math.pi/2)
        # append CameraMesh with its textures
        self.append_meshes(['CameraMesh'], "camera")
        self.rotate(z=math.pi)
    def rotate(self, x=0, y=0, z=0):
        SensorCreator.rotate(self, x=y, y=z, z=x)

class DepthCamera(VideoCamera):
    _classpath = "morse.sensors.depth_camera.DepthCamera"
    _blendname = "camera"

    def __init__(self, name=None):
        VideoCamera.__init__(self, name)
        self.properties(cam_near=1.0, cam_far=20.0, retrieve_depth=True,
                        Vertical_Flip=False)

class Velodyne(DepthCamera):
    _classpath = "morse.sensors.depth_camera.DepthCameraRotationZ"

    def __init__(self, name=None):
        DepthCamera.__init__(self, name)
        self.properties(rotation=self.camera._bpy_object.data.angle)

VelodyneZB = Velodyne # morse 1.1 compatible

class SemanticCamera(VideoCamera):
    _classpath = "morse.sensors.semantic_camera.SemanticCamera"
    _blendname = "camera"

    def __init__(self, name=None):
        VideoCamera.__init__(self, name)
        self.properties(cam_width = 512, cam_height = 512, Vertical_Flip = False)


class VelodyneRayCast(LaserSensorWithArc):
    _classpath = "morse.sensors.laserscanner.LaserScannerRotationZ"
    _blendname = "velodyne"

    def __init__(self, name=None):
        LaserSensorWithArc.__init__(self, name)
        # set components-specific properties
        self.properties(Visible_arc = True, laser_range = 50.0,
                        scan_window = 31.500, resolution = 0.5)
        # append velodyne mesh, from MORSE_COMPONENTS/sensors/velodyne.blend
        arc = self.create_laser_arc()
        # Select only arc (active)
        bpymorse.select_only(arc)
        # Rotate Arc to scan vertically
        arc.rotation_euler = (math.radians(90), math.radians(12), 0)
        bpymorse.apply_transform(rotation=True)
        self.append_meshes(['VelodyneMesh'])

class Clock(SensorCreator):
    _classpath = "morse.sensors.clock.Clock"

    def __init__(self, name=None):
        SensorCreator.__init__(self, name)

class Kinect(SensorCreator):
    _classpath = "morse.core.sensor.Sensor"

    def __init__(self, name="Kinect"):
        # meta sensor composed of 2 cameras (rgb and depth)
        SensorCreator.__init__(self, name)
        mesh = Cube("KinectMesh")
        mesh.scale = (.02, .1, .02)
        mesh.color(.8, .8, .8)
        self.append(mesh)
        self.video_camera = VideoCamera('rgb')
        self.video_camera.properties(cam_width = 128, cam_height=128)
        self.depth_camera = DepthCamera('depth')
        self.depth_camera.properties(classpath='morse.sensors.depth_camera.DepthVideoCamera')
        self.depth_camera.properties(cam_width = 128, cam_height=128, Vertical_Flip=True)
        self.append(self.video_camera)
        self.append(self.depth_camera)
        # TODO find Kinect spec for cameras positions
        self.video_camera.location = (.06, +.08, .04)
        self.depth_camera.location = (.06, -.08, .04)
    def add_stream(self, *args, **kwargs):
        # Override AbstractComponent method
        self.video_camera.add_stream(*args, **kwargs)
        self.depth_camera.add_stream(*args, **kwargs)
    def profile(self):
        # Override AbstractComponent method
        self.video_camera.profile()
        self.depth_camera.profile()
    def frequency(self, frequency):
        # Override AbstractComponent method
        self.video_camera.frequency(frequency)
        self.depth_camera.frequency(frequency)

class Collision(SensorCreator):
    _classpath = "morse.sensors.collision.Collision"

    def __init__(self, name=None):
        """ Sensor to detect objects colliding with the current object,
        with more settings than the Touch sensor
        """
        SensorCreator.__init__(self, name)
        obj = bpymorse.get_context_object()
        # Sensor, Collision Sensor, detects static and dynamic objects but
        # not the other collision sensor objects.
        obj.game.physics_type = 'SENSOR'
        # Specify a collision bounds type other than the default
        obj.game.use_collision_bounds = True
        # replace Always sensor by Collision sensor
        sensor = obj.game.sensors[-1]
        sensor.type = 'COLLISION'
        # need to get the new Collision Sensor object
        sensor = obj.game.sensors[-1]
        sensor.use_pulse_true_level = True # FIXME doesnt seems to have any effect
        # Component mesh (eye sugar)
        mesh = Cube("CollisionMesh")
        mesh.scale = (.02, .02, .02)
        mesh.color(.8, .2, .1)
        self.append(mesh)
    def properties(self, **kwargs):
        SensorCreator.properties(self, **kwargs)
        if 'collision_property' in kwargs:
            try:
                sensor = self._bpy_object.game.sensors[-1]
                sensor.property = kwargs['collision_property']
            except KeyError:
                pass
