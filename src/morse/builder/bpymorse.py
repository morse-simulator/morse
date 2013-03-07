""" This module wraps the calls to the Blender Python API. This is intended
for all the cases we need to run MORSE code outside Blender (mostly for
documentation generation purposes).
"""
from morse.core.exceptions import MorseBuilderNoComponentError

bpy = None

try:
    import bpy
except ImportError:
    print("ATTENTION: MORSE is running outside Blender! (no bpy)")

def empty_method(*args, **kwargs):
    print(args, kwargs)

select_all = empty_method
add_mesh_monkey = empty_method
add_mesh_plane = empty_method
add_mesh_cube = empty_method
add_mesh_uv_sphere = empty_method
add_mesh_ico_sphere = empty_method
add_mesh_cylinder = empty_method
add_mesh_cone = empty_method
add_mesh_torus = empty_method
add_lamp = empty_method
add_camera = empty_method
new_material = empty_method
new_text = empty_method
new_game_property = empty_method
add_sensor = empty_method
add_controller = empty_method
add_actuator = empty_method
link_append = empty_method
collada_import = empty_method
add_object = empty_method
add_empty = empty_method
new_mesh = empty_method
new_object = empty_method
apply_transform = empty_method

if bpy:
    select_all = bpy.ops.object.select_all
    add_mesh_monkey = bpy.ops.mesh.primitive_monkey_add
    add_mesh_plane = bpy.ops.mesh.primitive_plane_add
    add_mesh_cube = bpy.ops.mesh.primitive_cube_add
    add_mesh_uv_sphere = bpy.ops.mesh.primitive_uv_sphere_add
    add_mesh_ico_sphere = bpy.ops.mesh.primitive_ico_sphere_add
    add_mesh_cylinder = bpy.ops.mesh.primitive_cylinder_add
    add_mesh_cone = bpy.ops.mesh.primitive_cone_add
    add_mesh_torus = bpy.ops.mesh.primitive_torus_add
    add_lamp = bpy.ops.object.lamp_add
    add_camera = bpy.ops.object.camera_add
    new_material = bpy.ops.material.new
    new_text = bpy.ops.text.new
    new_game_property = bpy.ops.object.game_property_new
    add_sensor = bpy.ops.logic.sensor_add
    add_controller = bpy.ops.logic.controller_add
    add_actuator = bpy.ops.logic.actuator_add
    link_append = bpy.ops.wm.link_append
    collada_import = bpy.ops.wm.collada_import
    add_object = bpy.ops.object.add
    if bpy.app.version >= (2, 65, 0):
        add_empty = bpy.ops.object.empty_add
    new_mesh = bpy.data.meshes.new
    new_object = bpy.data.objects.new
    apply_transform = bpy.ops.object.transform_apply

def create_new_material():
    all_materials = get_materials().keys()
    new_material()
    material_name = [name for name in get_materials().keys() \
                     if name not in all_materials].pop()
    return get_material(material_name)

def add_morse_empty():
    """Add MORSE Component Empty object which hlods MORSE logic"""
    if bpy.app.version >= (2, 65, 0):
        add_empty(type='ARROWS')
    else:
        add_object(type='EMPTY')

def deselect_all():
    select_all(action='DESELECT')

def get_first_selected_object():
    if bpy and bpy.context.selected_objects:
        return bpy.context.selected_objects[0]
    else:
        return None

def get_selected_objects():
    if bpy:
        return bpy.context.selected_objects
    else:
        return []

def get_lamps():
    if bpy:
        return bpy.data.lamps
    else:
        return []

def get_lamp(name_or_id):
    if bpy and bpy.data.lamps:
        return bpy.data.lamps[name_or_id]
    else:
        return None

def get_last_lamp():
    return get_lamp(-1)

def get_materials():
    if bpy:
        return bpy.data.materials
    else:
        return []

def get_material(name_or_id):
    if bpy and bpy.data.materials:
        return bpy.data.materials[name_or_id]
    else:
        return None

def get_last_material():
    return get_material(-1)

def get_texts():
    if bpy:
        return bpy.data.texts
    else:
        return []

def get_text(name_or_id):
    if bpy and bpy.data.texts:
        return bpy.data.texts[name_or_id]
    else:
        return None

def get_last_text():
    return get_text(-1)

def select_only(obj):
    if bpy:
        deselect_all()
        obj.select = True
        bpy.context.scene.objects.active = obj

def get_objects():
    if bpy:
        return bpy.data.objects
    else:
        return []

def get_object(name_or_id):
    if bpy and bpy.data.objects:
        return bpy.data.objects[name_or_id]
    else:
        return None

def get_fps():
    if bpy:
        return bpy.context.scene.game_settings.fps
    else:
        return -1

def get_context_object():
    if bpy:
        return bpy.context.object
    else:
        return None

def get_context_scene():
    if bpy:
        return bpy.context.scene
    else:
        return None

def get_context_window():
    if bpy:
        return bpy.context.window
    else:
        return None

def set_debug(debug=True):
    bpy.app.debug = debug

def get_objects_in_blend(filepath):
    if not bpy:
        return []
    objects = []
    try:
        with bpy.data.libraries.load(filepath) as (src, _):
            try:
                objects = [obj for obj in src.objects]
            except UnicodeDecodeError as detail:
                logger.error("Unable to open file '%s'. Exception: %s" % \
                             (filepath, detail))
    except IOError as detail:
        logger.error(detail)
        raise MorseBuilderNoComponentError("Component not found")
    return objects

def save(filepath=None, check_existing=False, compress=True):
    """ Save .blend file

    :param filepath: File Path
    :type  filepath: string, (optional, default: current file)
    :param check_existing: Check and warn on overwriting existing files
    :type  check_existing: boolean, (optional, default: False)
    :param compress: Compress, Write compressed .blend file
    :type  compress: boolean, (optional, default: True)
    """
    if not bpy:
        return
    if not filepath:
        filepath = bpy.data.filepath
    bpy.ops.wm.save_mainfile(filepath=filepath, check_existing=check_existing,
            compress=compress)

def set_speed(fps=0, logic_step_max=0, physics_step_max=0):
    """ Tune the speed of the simulation

    :param fps: Nominal number of game frames per second
        (physics fixed timestep = 1/fps, independently of actual frame rate)
    :type fps: int in [1, 250], default 0
    :param logic_step_max: Maximum number of logic frame per game frame if
        graphics slows down the game, higher value allows better
        synchronization with physics
    :type logic_step_max: int in [1, 5], default 0
    :param physics_step_max: Maximum number of physics step per game frame
        if graphics slows down the game, higher value allows physics to keep
        up with realtime
    :type physics_step_max: int in [1, 5], default 0

    usage::

        bpymorse.set_speed(120, 5, 5)

    .. warning:: This method must be called at the top of your Builder script,
      before creating any component.
    """
    get_context_scene().game_settings.fps = fps
    get_context_scene().game_settings.logic_step_max = logic_step_max
    get_context_scene().game_settings.physics_step_max = physics_step_max

