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
open_sound = empty_method
new_scene = empty_method
armatures = empty_method

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
    open_sound = bpy.ops.sound.open
    new_scene = bpy.ops.scene.new
    armatures = bpy.data.armatures

def version():
    if bpy:
        return bpy.app.version
    else:
        return 0,0,0


def create_new_material():
    all_materials = get_materials().keys()
    new_material()
    material_name = [name for name in get_materials().keys() \
                     if name not in all_materials].pop()
    return get_material(material_name)

def add_morse_empty(shape = 'ARROWS'):
    """Add MORSE Component Empty object which hlods MORSE logic"""
    if bpy.app.version >= (2, 65, 0):
        add_empty(type = shape)
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

def get_sounds():
    if bpy:
        return bpy.data.sounds
    else:
        return []

def get_sound(name_or_id):
    if bpy and bpy.data.sounds:
        return bpy.data.sounds[name_or_id]
    else:
        return None

def get_last_sound():
    return get_sound(-1)

def get_scenes():
    if bpy:
        return bpy.data.scenes
    else:
        return []

def get_scene(name_or_id):
    if bpy and bpy.data.scenes:
        return bpy.data.scenes[name_or_id]
    else:
        return None

def set_active_scene(name_or_id):
    if bpy:
        scene = get_scene(name_or_id)
        if scene:
            bpy.data.screens['Default'].scene = scene
            bpy.context.screen.scene = scene
            return scene
        else:
            return None
    else:
        return None

def get_last_scene():
    return get_scene(-1)

def select_only(obj):
    if bpy:
        deselect_all()
        obj.select = True
        bpy.context.scene.objects.active = obj

def delete(objects):
    if not bpy:
        return
    if not isinstance(objects, list):
        objects = [objects]
    for obj in objects:
        if isinstance(obj, str):
            obj = bpy.data.objects[obj]
        select_only(obj)
        bpy.ops.object.delete()

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

def get_properties(obj):
    retvalue = {}
    for name, prop in obj.game.properties.items():
        retvalue[name] = prop.value
    return retvalue

def properties(obj, **kwargs):
    """ Add/modify the game properties of the Blender object

    Usage example:

    .. code-block:: python

        properties(obj, capturing = True, classpath='module.Class', speed = 5.0)

    will create and/or set the 3 game properties Component_Tag, classpath, and
    speed at the value True (boolean), 'module.Class' (string), 5.0 (float).
    In Python the type of numeric value is 'int', if you want to force it to
    float, use the following: float(5) or 5.0
    Same if you want to force to integer, use: int(a/b)
    For the TIMER type, see the class timer(float) defined in this module:

    .. code-block:: python

        properties(obj, my_clock = timer(5.0), my_speed = int(5/2))

    """
    for key in kwargs.keys():
        if key in obj.game.properties.keys():
            _property_set(obj, key, kwargs[key])
        else:
            _property_new(obj, key, kwargs[key])

def _property_new(obj, name, value, ptype=None):
    """ Add a new game property for the Blender object

    :param name: property name (string)
    :param value: property value
    :param ptype: property type (enum in ['BOOL', 'INT', 'FLOAT', 'STRING', 'TIMER'],
                  optional, auto-detect, default=None)
    """
    select_only(obj)
    new_game_property()
    # select the last property in the list (which is the one we just added)
    obj.game.properties[-1].name = name
    return _property_set(obj, -1, value, ptype)

def _property_set(obj, name_or_id, value, ptype=None):
    """ Really set the property for the property referenced by name_or_id

    :param name_or_id: the index or name of property (OrderedDict)
    :param value: the property value
    :param ptype: property type (enum in ['BOOL', 'INT', 'FLOAT', 'STRING', 'TIMER'],
                  optional, auto-detect, default=None)
    """
    if ptype is None:
        # Detect the type (class name upper case)
        ptype = value.__class__.__name__.upper()
    if ptype == 'STR':
        # Blender property string are called 'STRING' (and not 'str' as in Python)
        ptype = 'STRING'
    obj.game.properties[name_or_id].type = ptype
    obj.game.properties[name_or_id].value = value
    return obj.game.properties[name_or_id]

def set_viewport(viewport_shade='WIREFRAME', clip_end=1000):
    """ Set the default view mode

    :param viewport_shade: enum in ['BOUNDBOX', 'WIREFRAME', 'SOLID', 'TEXTURED'], default 'WIREFRAME'
    """
    for area in bpy.context.window.screen.areas:
        if area.type == 'VIEW_3D':
            for space in area.spaces:
                if space.type == 'VIEW_3D':
                    space.viewport_shade = viewport_shade
                    space.clip_end = clip_end

def set_viewport_perspective(perspective='CAMERA'):
    """ Set the default view view_perspective

    Equivalent to ``bpy.ops.view3d.viewnumpad`` with good context.

    :param perspective: View, Preset viewpoint to use
    :type  perspective: enum in ['FRONT', 'BACK', 'LEFT', 'RIGHT', 'TOP',
                                 'BOTTOM', 'CAMERA'], default 'CAMERA'
    """
    for area in bpy.context.window.screen.areas:
        if area.type == 'VIEW_3D':
            for space in area.spaces:
                if space.type == 'VIEW_3D':
                    space.region_3d.view_perspective = 'CAMERA'

def fullscreen(fullscreen=True):
    """ Run the simulation fullscreen

    :param fullscreen: Start player in a new fullscreen display
    :type  fullscreen: Boolean, default: True
    """
    if not bpy:
        return
    bpy.context.scene.game_settings.show_fullscreen = fullscreen
