# usage: blender file.blend -P builder_generator.py > file.py
import bpy, json

# helpers (c/p following in a Blender Text editor, run it, look at your console)

def select_only(obj):
    bpy.ops.object.select_all(action='DESELECT')
    obj.select = True
    bpy.context.scene.objects.active = obj

def roundv(v,r=3):
    return tuple([round(e, r) for e in v])

def do_all():
    print("base = AbstractComponent(category='props', filename='basics_wo_logic')")
    meshes = [o.name for o in bpy.data.objects if o.type == 'MESH']
    print("base.append_meshes(%s)"%repr(meshes))
    for obj in bpy.data.objects:
        print("# %s : %s"%(obj.type, obj.name))
        if obj.type == 'MESH':
            print("select_only(bpy.data.objects['%s'])"%obj.name)
        else:
            print("bpy.ops.object.add(type='%s', location=%s, rotation=%s)" % \
                    (obj.type, repr(obj.location), repr(obj.rotation_euler)))
            print("bpy.context.object.name = '%s'" % obj.name)
            if obj.scale != (1,1,1):
                print("bpy.context.object.scale = %s" % repr(obj.scale))

        print("bpy.context.object.game.physics_type = '%s'" % obj.game.physics_type)
        print("bpy.context.object.hide_render = %s" % obj.hide_render)
        print_properties(obj)
        print_all_logic(obj)

    # set children - parent relationship
    for obj in bpy.data.objects:
        for child in obj.children:
            # child.parent = obj
            print("bpy.data.objects['%s'].parent = bpy.data.objects['%s']" % \
                (child.name, obj.name) )

    # set children - parent relationship
    for text in bpy.data.texts:
        print("# text : %s"%text.name)
        print("new_text()")
        print("text = get_last_text()")
        print("text.name = '%s'" % text.name)
        print("text_str = %s" % json.dumps(text.as_string()) )
        print("text.write(text_str)")


def get_properties(obj):
    import json
    map_prop = {}
    properties = obj.game.properties
    for name in properties.keys():
        map_prop[name] = properties[name].value
        if properties[name].type == 'TIMER':
            map_prop[name] = "timer(%f)"%map_prop[name]
        elif type(map_prop[name]) is str:
            map_prop[name] = json.dumps(map_prop[name])
    return map_prop

def print_properties(obj):
    """ Returns the Game properties of the Blender objects """
    map_prop = get_properties(obj)
    if not map_prop:
        return
    str_prop = ", ".join(["%s=%s"%(pname, map_prop[pname]) \
        for pname in map_prop.keys()])
    print("properties(bpy.context.object, %s)" % str_prop)

def print_all_logic(obj):
    print("game = bpy.context.object.game")
    for brick in obj.game.actuators:
        print_logic(brick, 'actuator')
    for brick in obj.game.controllers:
        print_logic(brick, 'controller')
        for act in brick.actuators:
            print("game.controllers['%s'].link(actuator=game.actuators['%s'])" % \
                (brick.name, act.name) )
    for brick in obj.game.sensors:
        print_logic(brick, 'sensor')
        for ctr in brick.controllers:
            print("game.sensors['%s'].link(game.controllers['%s'])" % \
                (brick.name, ctr.name) )

# map the default values of game logic bricks to filter the output
map_attr_default = {
    'default': {
        '__module__': '',
        'type': '',
        #'name': '',
        'show_expanded': '',
    },
    'sensor': {
        'default': {
            'frequency': 0,
            'invert': False,
            'use_level': False,
            'use_pulse_false_level': False,
            'use_pulse_true_level': False,
            'use_tap': False,
            'pin': False,
        },
        'PROPERTY': {
            'evaluation_type': 'PROPEQUAL',
            'property': '',
            'value': '',
            'value_max': '',
            'value_min': '',
        },
        'KEYBOARD': {
            'log': '',
            'modifier_key_1': 'NONE',
            'modifier_key_2': 'NONE',
            'target': '',
            'use_all_keys': False,
        },
        'MOUSE': {
            'use_pulse': False,
        },
    },
    'controller': {
        'default': {
            'use_priority': False,
            'states': 1,
        },
        'PYTHON': {
            'mode': 'SCRIPT',
            'module': '',
            'use_debug': False,
        },
    },
    'actuator': {
        'default': {
            'pin': False,
        },
        'VISIBILITY': {
            'apply_to_children': False,
            'use_occlusion': False,
        },
    },
}

def print_logic(brick, sca):
    print("add_%s(type='%s', name='%s')"%(sca, brick.type, brick.name))

    for attr in dir(brick):
        if attr in map_attr_default['default'].keys():
            continue # skip
        value = getattr(brick, attr)
        if attr in map_attr_default[sca]['default'].keys() and \
            value == map_attr_default[sca]['default'][attr]:
            continue # skip default to save readibility
        if brick.type in map_attr_default[sca] and \
            attr in map_attr_default[sca][brick.type].keys() and \
            value == map_attr_default[sca][brick.type][attr]:
            continue # skip default to save readibility
        if type(value).__name__ in ['int', 'float', 'bool']:
            print("game.%ss[-1].%s = %s" % (sca, attr, str(value)) )
        if type(value).__name__ == 'str':
            print("game.%ss[-1].%s = %s" % (sca, attr, json.dumps(value)) )

_header = """# usage: blender -P {file}
import os, sys, subprocess

def ext_exec(cmd, python=None):
    if not python:
        python = 'python%i.%i'%(sys.version_info.major, sys.version_info.minor)
    return subprocess.getoutput('%s -c"%s"' % (python, cmd) )

def fix_python_path(python=None):
    pythonpath = ext_exec("import os,sys;print(os.pathsep.join(sys.path))")
    sys.path.extend(pythonpath.split(os.pathsep))

fix_python_path()

from morse.builder.bpymorse import *
from morse.builder import AbstractComponent
from mathutils import Vector, Euler
"""

if __name__ == '__main__':
    print(_header)
    do_all()
    print("\n\n# TODO remove the following lines (Blender output messages)\n\n")
