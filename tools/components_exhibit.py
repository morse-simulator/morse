import os
import sys
import pkgutil
import inspect

try:
    import bpy # Blender
except ImportError:
    print("Could not import bpy, run with Blender\n"
          "blender -P components_exhibit.py")
    sys.exit(-1)
from mathutils import Euler

try:
    import morse # MORSE
except ImportError:
    raise ImportError("Could not import morse, set your PYTHONPATH\n"
        'export PYTHONPATH="/usr/local/lib/python3.3/site-packages/:$PYTHONPATH"')
from morse.builder import *

#
# helpers
#

def get_classes_from_module(module_name):
    __import__(module_name)
    # Predicate to make sure the classes only come from the module in question
    def predicate(member):
        return inspect.isclass(member) and member.__module__.startswith(module_name)
    # fetch all members of module name matching 'pred'
    return inspect.getmembers(sys.modules[module_name], predicate)

def get_submodules(module_name):
    """ Get a list of submodules from a module name.
    Not recursive, don't return nor look in subpackages """
    __import__(module_name)
    module = sys.modules[module_name]
    module_path = getattr(module, '__path__')
    return [name for _, name, ispkg in pkgutil.iter_modules(module_path) if not ispkg]

def get_subclasses(module_name, skip_submodules=[]):
    subclasses = []
    submodules = get_submodules(module_name)
    for submodule_name in submodules:
        if submodule_name in skip_submodules:
            pass
        submodule = "%s.%s"%(module_name, submodule_name)
        try:
            submodule_classes = get_classes_from_module(submodule)
            for _, klass in submodule_classes:
                subclasses.append(klass)
        except Exception:
            # can not import some resources
            pass
    return subclasses

modules = [
    "morse.builder.sensors",
    "morse.builder.actuators",
    "morse.builder.robots",
]

specific_camera_pose_per_component = {
    # Robots
    'BasePR2': (.36, -2.25, 1.77),
    'Morsy': (3.84, -3.12, 2.12, 1.32, 0.0, 0.86),
    'SegwayRMP400': (.36, -2.25, 1.77),
    'ATRV': (.36, -2.25, 1.77),
    'Human': (4, 0, 2.8), # 65°,0,90°
    'Hummer': (.36, -2.25, 1.77),
    'Jido': (.36, -2.25, 1.77),
    'B21': (.36, -2.25, 1.77),
    'RMax': (.36, -2.25, 1.77),
    'Submarine': (.36, -2.25, 1.77),
    'Vicitm': (.36, -2.25, 1.77),
    'NavPR2': (.36, -2.25, 1.77),
    'QUAD2012': (.36, -1.25, .77),
    'Quadrotor': (.36, -1.25, .77),
    'Pioneer3DX': (.36, -1.25, .77),
    # Sensors
    'Accelerometer': (.36, -1.25, .77),
    'Battery': (.36, -1.25, .77),
    'DepthCamera': (.36, -1.25, .77),
    'GPS': (.36, -1.25, .77),
    'Gyroscope': (.36, -1.25, .77),
    'Hokuyo': (.36, -1.25, .77),
    'IMU': (.36, -1.25, .77),
    'Infrared': (.36, -1.25, .77),
    'Kinect': (.36, -1.25, .77),
    'Odometry': (.36, -1.25, .77),
    'Pose': (.36, -1.25, .77),
    'Proximity': (.36, -1.25, .77),
    'SearchAndRescue': (.36, -1.25, .77),
    'SemanticCamera': (.36, -1.25, .77),
    'Sick': (.36, -1.25, .77),
    'SickLDMRS': (.36, -1.25, .77),
    'StereoUnit': (.36, -1.25, .77),
    'Thermometer': (.36, -1.25, .77),
    'VideoCamera': (.36, -1.25, .77),
    'Velodyne': (.36, -1.25, .77),
    # Actuators
    'PTU': (.36, -1.25, .77),
    'PA10': (.36, -2.25, 1.77),
    'KukaLWR': (.36, -2.25, 1.77),
    'Gripper': (.36, -1.25, .77),
}

def pose_camera(location = (.36, -1.25, .77), rotation = (1.1, 0, 0.25)):
    if len(location) == 6:
        rotation = location[3:]
        location = location[:3]
    scene = bpy.context.scene
    scene.camera.rotation_euler = Euler(rotation, 'XYZ')
    scene.camera.location = location

def delete(obj):
    bpymorse.select_only(obj)
    bpy.ops.object.delete()

def setup_scene():
    scene = bpy.context.scene
    if 'Cube' in bpy.data.objects:
        delete(bpy.data.objects['Cube'])
    # Set the scene's camera
    scene.camera = bpy.data.objects['Camera']
    # Set the scene's output file format
    scene.render.image_settings.file_format = 'PNG'
    # RGBA, Images are saved with RGB and Alpha data (if supported).
    scene.render.image_settings.color_mode = 'RGBA'
    # Premultiplied, Transparent RGB pixels are multiplied by the alpha channel.
    scene.render.alpha_mode = 'PREMUL'
    # Move the default Lamp
    bpy.data.objects['Lamp'].location = (-2, 3, 6)
    if len(bpy.data.lamps) < 3:
        # Add new lamp point
        bpy.ops.object.lamp_add(type='POINT', location=(5, 2, 1))
        bpy.data.lamps[-1].energy = 0.6
        bpy.ops.object.lamp_add(type='POINT', location=(-4, 3, 4))
        bpy.data.lamps[-1].energy = 0.4
    # Move the default Camera
    pose_camera()

def render_component(klass, save_path):
    scene = bpy.context.scene
    class_name = klass.__name__
    if class_name in specific_camera_pose_per_component:
        pose_camera(specific_camera_pose_per_component[class_name])
    else:
        print("#### class not in dict: %s"%str(class_name))
        pose_camera()
    origin_objects = [obj.name for obj in bpy.data.objects]
    # load class
    component = klass()
    # Refresh the view
    bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
    # Render the scene
    scene.render.filepath = os.path.join(save_path, klass.__name__ + ".png")
    bpy.ops.render.render(write_still=True)
    # Remove imported objects
    component.select() # select only the compoenent
    # Select new component (its children)
    for obj in bpy.data.objects:
        if obj.name not in origin_objects:
            obj.select = True
    # Remove the inserted objects
    bpy.ops.object.delete()

def main():
    """ Generate "studio" image of MORSE components

    HOWTO

    export PYTHONPATH="/usr/local/lib/python3.3/site-packages/:$PYTHONPATH"
    blender -P components_exhibit.py

    Et voila!
    """
    print ("==== PHOTO STUDIO FOR MORSE COMPONENTS ====\n\n")
    save_path = "//morse_renders"
    scene = bpy.context.scene
    print('Using Scene[%s]'%scene.name)
    setup_scene()

    # browse morse components modules
    for module in modules:
        print("browse %s classes"%module)
        for _, klass in get_classes_from_module(module):
            print("process %s"%str(klass))
            if issubclass(klass, AbstractComponent):
                try:
                    render_component(klass, save_path)
                except Exception as e:
                    print("ERROR : Could not render %s : %s"%(str(klass), str(e)))
            else:
                print("ERROR: Not a Component : %s"%str(klass))

            print("\n\n")

    print ("DONE!")


if __name__ == '__main__':
    main()

