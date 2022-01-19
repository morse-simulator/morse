import logging; logger = logging.getLogger("morse." + __name__)
from morse.core import mathutils
import morse.core.sensor
from morse.core.services import service, async_service
from morse.core import status, blenderapi
from morse.helpers.components import add_data, add_property
from morse.builder import bpymorse
from morse.core.mathutils import *
import numpy as np
import bmesh
import bpy
import capnp
import json
from queue import Queue
import time
import math

import sys
sys.path.extend(["/usr/local/share/", "/usr/local/share/cortex"])
cortex = None
try:
    import cortex_capnp as cortex
except:
    print("\033[1;31m")
    print("Cortex could not be found!")
    print("\033[0m")

flatten = lambda l: [item for sublist in l for item in sublist]

# Set logger level - DEBUG used for timing of all messages sent
# logger.setLevel(logging.INFO)

def create_trigger_msg(position, rotation, dim_x, dim_y, dim_z, dict_msg = True):
    if not isinstance(rotation, Quaternion):
        rotation = rotation.to_quaternion()
    if dict_msg:
        launch_trigger = {
           'pose': {
                'position' : [position.x, position.y, position.z],
                'rotation' : {
                    'w': rotation.w,
                    'x': rotation.x,
                    'y': rotation.y,
                    'z': rotation.z
                }
           },
           'dimensions'    : [dim_x, dim_y, dim_z]
        }
    else:
        launch_trigger = cortex.LaunchTrigger.new_message()
        launch_trigger.pose.position.x = position.x
        launch_trigger.pose.position.y = position.y
        launch_trigger.pose.position.z = position.z
        launch_trigger.pose.rotation.w = rotation.w
        launch_trigger.pose.rotation.x = rotation.x
        launch_trigger.pose.rotation.y = rotation.y
        launch_trigger.pose.rotation.z = rotation.z
        launch_trigger.dimensions.x = dim_x
        launch_trigger.dimensions.y = dim_y
        launch_trigger.dimensions.z = dim_z
    return launch_trigger

def create_instance_msg(optix_instance, optix_object, dict_msg = True):
    position = optix_instance.worldPosition
    rotation = optix_instance.worldOrientation.to_quaternion()
    scale = optix_object.scale
    properties = optix_object.game.properties
    has_rgba = 'texture' in properties and properties['texture'].value and len(optix_object.data.materials) > 0
    rgba_texture_name = "" if not has_rgba else optix_object.data.materials[0].texture_slots[0].texture.name
    rgba_uvs_name = "" if not has_rgba else optix_object.data.name

    if dict_msg:
        # Core properties
        instance = {
            'instance_name' : optix_instance.name,
            'mesh_name' : optix_object.data.name,
            'transform': {
                'position' : [position.x, position.y, position.z],
                'rotation' : {
                    'w': rotation.w,
                    'x': rotation.x,
                    'y': rotation.y,
                    'z': rotation.z
                },
                'scale' :    [scale.x, scale.y, scale.z]
            }
        }
        if (has_rgba):
            instance['texture_descriptions'] = [{'texture_identifier': rgba_texture_name,
                                                'uvs_identifier': rgba_uvs_name,
                                                'texture_type': 'RGBA_TEXTURE'}]

        # Extra properties
        try:
            angular_velocity = optix_instance.worldAngularVelocity
            instance['angular_velocity'] = [angular_velocity.x, angular_velocity.y, angular_velocity.z]
        except: pass
        try:
            linear_velocity = optix_instance.worldLinearVelocity
            instance['linear_velocity'] = [linear_velocity.x, linear_velocity.y, linear_velocity.z]
        except: pass
        if 'density' in properties:
            if properties['density'].type == 'FLOAT':
                instance['density'] = properties['density'].value
            else:
                logger.warn('density should be of type FLOAT but was ' + properties['density'].type)
        instance['reflectance'] = {}
        if 'Kd' in properties:
            instance['reflectance']['kd'] = properties['Kd'].value
        else:
            instance['reflectance']['kd'] = 1.0
        if 'Ks' in properties:
            instance['reflectance']['ks'] = properties['Ks'].value
        else:
            instance['reflectance']['ks'] = 0.0
        if 'sound_speed_compression' in properties:
            if properties['sound_speed_compression'].type == 'FLOAT':
                instance['sound_speed_compression'] = properties['sound_speed_compression'].value
            else:
                logger.warn('sound_speed_compression should be of type FLOAT but was ' + properties['sound_speed_compression'].type)
        return instance
    else:
        instance = cortex.Instance.new_message()

        # Core properties
        instance.instanceName              = optix_instance.name
        instance.meshName                  = optix_object.data.name
        if (has_rgba):
            instance_texture_descriptions                      = instance.init('textureDescriptions', 1)
            instance_texture_descriptions[0].textureIdentifier = rgba_texture_name
            instance_texture_descriptions[0].uvsIdentifier     = rgba_uvs_name
            instance_texture_descriptions[0].textureType       = cortex.TextureDescription.TextureType.rgbaTexture
        instance.transform.position.x      = position.x
        instance.transform.position.y      = position.y
        instance.transform.position.z      = position.z
        instance.transform.rotation.w      = rotation.w
        instance.transform.rotation.x      = rotation.x
        instance.transform.rotation.y      = rotation.y
        instance.transform.rotation.z      = rotation.z
        instance.transform.scale.x         = scale.x
        instance.transform.scale.y         = scale.y
        instance.transform.scale.z         = scale.z

        # Extra properties
        try:
            angular_velocity = optix_instance.worldAngularVelocity
            instance.angularVelocity.x = angular_velocity.x
            instance.angularVelocity.y = angular_velocity.y
            instance.angularVelocity.z = angular_velocity.z
        except: pass
        try:
            linear_velocity = optix_instance.worldLinearVelocity
            instance.linearVelocity.x  = linear_velocity.x
            instance.linearVelocity.y  = linear_velocity.y
            instance.linearVelocity.z  = linear_velocity.z
        except: pass
        if 'density' in properties:
            if properties['density'].type == 'FLOAT':
                instance.density = properties['density'].value
            else:
                logger.warn('density should be of type FLOAT but was ' + properties['density'].type)
        if 'Kd' in properties:
            instance.reflectance.kd = properties['Kd'].value
        else:
            instance.reflectance.kd = 1.0
        if 'Ks' in properties:
            instance.reflectance.ks = properties['Ks'].value
        else:
            instance.reflectance.ks = 0.0
        if 'sound_speed_compression' in properties:
            if properties['sound_speed_compression'].type == 'FLOAT':
                instance.soundSpeedCompression = properties['sound_speed_compression'].value
            else:
                logger.warn('sound_speed_compression should be of type FLOAT but was ' + properties['sound_speed_compression'].type)
        return instance

def create_light_base(lamp_object, dict_msg = True):
    decay_length = lamp_object.data.distance
    falloff_type = 'CONSTANT'
    try:
        falloff_type = lamp_object.data.falloff_type
    except:
        pass
    if dict_msg:
        decay_type = 'CONSTANT'
        # These all happen to be named identically but new decay types may be different, so check each
        if falloff_type == 'CONSTANT':
            decay_type = 'CONSTANT'
        elif falloff_type == 'INVERSE_LINEAR':
            decay_type = 'INVERSE_LINEAR'
        elif falloff_type == 'INVERSE_SQUARE':
            decay_type = 'INVERSE_SQUARE'
        else:
            logger.warn('Lamp decay type ' + str(falloff_type) + ' not supported by ObjectServer. Setting to CONSTANT')
        base = {
            'identifier': lamp_object.name,
            'strength': lamp_object.data.energy,
            'decay_type': decay_type,
            'decay_length': decay_length
        }
        return base
    else:
        decay_type = cortex.LightSourceBase.DecayType.constant
        # These all happen to be named identically but new decay types may be different, so check each
        if falloff_type == 'CONSTANT':
            decay_type = cortex.LightSourceBase.DecayType.constant
        elif falloff_type == 'INVERSE_LINEAR':
            decay_type = cortex.LightSourceBase.DecayType.inverseLinear
        elif falloff_type == 'INVERSE_SQUARE':
            decay_type = cortex.LightSourceBase.DecayType.inverseSquare
        elif falloff_type != 'NONE':
            logger.warn('Lamp decay type ' + str(falloff_type) + ' not supported by ObjectServer. Setting to constant')
        base = cortex.LightSourceBase.new_message()
        base.identifier = lamp_object.name
        base.strength = lamp_object.data.energy
        base.decayType = decay_type
        base.decayLength = decay_length
        return base

def create_ambient_light(light_name, light, dict_msg = True):
    energy = light.environment_energy
    # colour = light.environment_color
    decay_length = 0.0
    if dict_msg:
        light = {
            'source': {
                'identifier': light_name,
                'strength': energy,
                'decay_type': 'CONSTANT',
                'decay_length': decay_length
            }
            #'colour': colour
        }
        return light
    else:
        light = cortex.AmbientLightSource.new_message()
        light.source.identifier = light_name
        light.source.strength = energy
        light.source.decayType = cortex.LightSourceBase.DecayType.constant
        light.source.decayLength = decay_length
        # light.colour = colour
        return light

def create_directional_light(lamp_object, dict_msg = True):
    base = create_light_base(lamp_object, dict_msg)
    # No rotation of Sun in blender points in -Z, but we expect no rotation to point along X axis, so start with -Z
    direction = mathutils.Euler((0.0, math.radians(90), 0.0), 'XYZ').to_quaternion()
    direction.rotate(lamp_object.matrix_world.to_3x3().to_quaternion())
    if dict_msg:
        light = {
            'source': base,
            'direction': {
                'w': direction.w,
                'x': direction.x,
                'y': direction.y,
                'z': direction.z
            }
        }
        return light
    else:
        light = cortex.DirectionalLightSource.new_message()
        light.source = base
        light.direction.x = direction.x
        light.direction.y = direction.y
        light.direction.z = direction.z
        light.direction.w = direction.w
        return light

def create_omni_light(lamp_object, dict_msg = True):
    base = create_light_base(lamp_object, dict_msg)
    position = lamp_object.matrix_world.to_translation()
    if dict_msg:
        light = {
            'source': base,
            'position': [position.x, position.y, position.z]
        }
        return light
    else:
        light = cortex.OmniLightSource.new_message()
        light.source = base
        light.position.x = position.x
        light.position.y = position.y
        light.position.z = position.z
        return light

def triangulate_object(obj):

    me = obj.data
    # Get a BMesh representation
    bm = bmesh.new()
    bm.from_mesh(me)

    bmesh.ops.triangulate(bm, faces=bm.faces[:], quad_method=0, ngon_method=0)

    # Finish up, write the bmesh back to the mesh
    bm.to_mesh(me)
    bm.free()

def fill_mesh(mesh, optix_obj):
    # Fill mesh
    mesh.identifier = optix_obj.data.name
    mesh.vertices = flatten([list(v.co) for v in optix_obj.data.vertices])
    mesh.faces = flatten([list(f.vertices) for f in optix_obj.data.polygons])

def fill_texture(texture, optix_texture):
    if (optix_texture.image.channels != 4):
        logger.error('expected four channel images: got %s' % optix_texture.image.channels)
    texture.identifier = optix_texture.name
    texture.dimensions.x = optix_texture.image.size[0]
    texture.dimensions.y = optix_texture.image.size[1]
    texture.dimensions.z = optix_texture.image.channels
    texture.data = optix_texture.image.pixels[:]

def fill_uvs(uvs, optix_obj):
    # Get per-vertex uvs
    uvs_np = np.zeros((len(optix_obj.data.loops), 2))
    uv_layer = optix_obj.data.uv_layers.active.data
    for loop in optix_obj.data.loops:
        uvs_np[loop.index, :] = uv_layer[loop.index].uv
    uvs.identifier = optix_obj.data.name
    uvs.data = uvs_np.flatten().tolist() # uvs.tolist() not provably faster

class Objectserver(morse.core.sensor.Sensor):

    _name = 'Objectserver'
    _short_desc = 'MOOS server to publish serialised object data'

    # add_data('scene_query',[],'list','Queue for query strings')
    add_data('inventory_requests', Queue(), 'queue', 'Queue for inventory requests')
    add_data('inventory_responses', Queue(), 'queue', 'Queue for inventory responses')
    add_data('inventory_updates', None, 'inventory', 'Inventory for inventory updates')
    add_data('mesh_requests', Queue(), 'queue', 'Queue for mesh requests')
    add_data('texture_requests', Queue(), 'queue', 'Queue for texture requests')
    add_data('mesh_responses', Queue(), 'queue', 'Queue for mesh responses')
    add_data('texture_responses', {}, 'dictionary', 'Dictionary of Queues for texture responses, where key is textureType')
    add_data('uvs_responses', {}, 'dictionary', 'Dictionary of Queues for uvs responses, where key is textureType')

    add_property('send_json', True, 'send_json',  'bool', 'Send small messages as json')

    def __init__(self, obj, parent=None):

        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)
        
        scene = blenderapi.scene()
        world = bpymorse.get_context_scene().world

        # All bge objects in scene
        bge_objs = scene.objects

        # All bpy objects in scene
        bpy_objs = bpymorse.get_objects()

        # Environmental ambient light
        environment_light = world.light_settings
        self.ambient_lights = {}
        if environment_light.use_environment_light:
            self.ambient_lights['world_light'] = environment_light
        else:
            logger.info("Ambient light is disabled")

        # Data structures for optix
        self.dynamic_instances = []
        self.optix_instances = []
        self.optix_objects = {}
        self.optix_textures = {}
        self.optix_ignored_instances = []
        self.directional_lights = {}
        self.omni_lights = {}
        for obj in bpy_objs:
            props = obj.game.properties
            if 'optix' in props and props['optix'].value:
                if obj.type == 'LAMP':
                    lamp = obj.data
                    if lamp.type == 'SUN':
                        self.directional_lights[obj.name] = obj
                    elif lamp.type == 'POINT':
                        self.omni_lights[obj.name] = obj
                else:
                    self.optix_instances.append(bge_objs[obj.name])
                    if not obj.data.name in self.optix_objects:
                        self.optix_objects[obj.data.name] = obj
                        if 'texture' in props and props['texture'].value:
                            if len(obj.data.materials) == 0:
                                raise RuntimeError('texture was specified for ' + obj.name + ' but no materials were found')
                            if len(obj.data.materials[0].texture_slots) == 0:
                                raise RuntimeError('texture was specified for ' + obj.name + ' but no texture slots were found for the material')
                            self.optix_textures[obj.data.materials[0].texture_slots[0].texture.name] = obj.data.materials[0].texture_slots[0].texture
                    if 'dynamic' in props and props['dynamic'].value:
                        self.dynamic_instances.append(bge_objs[obj.name])
            else:
                self.optix_ignored_instances.append(obj.name)
        logger.info("Dynamic Instances: " + str(self.dynamic_instances))
        logger.info("Optix Instances: " + str(self.optix_instances))
        logger.info("Optix Objects: " + str(self.optix_objects))
        logger.info("Optix Textures: " + str(self.optix_textures))
        logger.info("Optix Ignored Instances: " + str(self.optix_ignored_instances))
        logger.info("Optix Ambient Lights: " + str(self.ambient_lights))
        logger.info("Optix Directional Lights: " + str(self.directional_lights))
        logger.info("Optix Omni Lights: " + str(self.omni_lights))

        # Check objects for triangulation
        for bge_obj in self.optix_instances:
            bpy_obj = bpy_objs[bge_obj.name]
            triangulated = True
            for p in bpy_obj.data.polygons:
                if len(p.vertices) > 3:
                    triangulated = False

            if not triangulated:
                logger.warning('WARNING: Object %s must be triangulated...' % (bpy_obj.name))
                triangulate_object(bpy_obj)

        logger.info('Found %d dynamic objects in scene' % len(self.dynamic_instances))
        logger.info('Component initialized, runs at %.2f Hz', self.frequency)

        # TODO: change these for ordered sets
        self.mesh_request_set = set() # set of strings
        self.texture_request_sets = {} # dictionary of sets of strings, keys are textureTypes
        self.uvs_request_sets = {} # dictionary of sets of strings, keys are textureTypes

    def default_action(self):

        # Convert mesh request queue to data structure for efficient implementation
        while not self.local_data['mesh_requests'].empty():
            self.mesh_request_set.add(self.local_data['mesh_requests'].get())

        # Convert texture request queue to data structure for efficient implementation
        while not self.local_data['texture_requests'].empty():
            # Read the object request
            texture_request = cortex.TextureRequest.from_bytes(self.local_data['texture_requests'].get())
            for texture_description in texture_request.textureDescriptions:
                tex_type = texture_description.textureType
                if tex_type not in self.texture_request_sets:
                    self.texture_request_sets[tex_type] = set()
                self.texture_request_sets[tex_type].add(texture_description.textureIdentifier)
                if tex_type not in self.uvs_request_sets:
                    self.uvs_request_sets[tex_type] = set()
                self.uvs_request_sets[tex_type].add(texture_description.uvsIdentifier)

        bpy_objs = bpymorse.get_objects()

        if not self.local_data['inventory_requests'].empty():
            pid = self.local_data['inventory_requests'].get()
            if self.send_json:
                inventory = {
                    'inventory': {
                        'instances': [],
                        'lights': {
                            'ambient_lights': [],
                            'directional_lights': [],
                            'omni_lights': [],
                            'spotlight_lights': []
                        }
                    },
                    'uid': pid
                }
                instances = inventory['inventory']['instances']
                lights = inventory['inventory']['lights']

                # Add the instances
                for i in range(len(self.optix_instances)):
                    instances.append(create_instance_msg(self.optix_instances[i], bpy_objs[self.optix_instances[i].name], True))
                for light_name, light in self.ambient_lights.items():
                    lights['ambient_lights'].append(create_ambient_light(light_name, light, True))
                for light in self.directional_lights.values():
                    lights['directional_lights'].append(create_directional_light(light, True))
                for light in self.omni_lights.values():
                    lights['omni_lights'].append(create_omni_light(light, True))
                self.local_data['inventory_responses'].put(inventory)
            else:
                # Create capnp unique inventory
                inventory = cortex.UniqueInventory.new_message()
                inventory.uid = pid
                instances = inventory.inventory.init('instances', len(self.optix_instances))
                lights = inventory.inventory.lights
                ambient_lights = lights.init('ambientLights', 0)
                directional_lights = lights.init('directionalLights', len(self.directional_lights))
                omni_lights = lights.init('omniLights', 0)
                spotlight_lights = lights.init('spotlightLights', 0)
                for i in range(len(self.optix_instances)):
                    instances[i] = create_instance_msg(self.optix_instances[i], bpy_objs[self.optix_instances[i].name], False)
                i = 0
                for light_name, light in self.ambient_lights.items():
                    ambient_lights[i] = create_ambient_light(light_name, light, False)
                    i += 1
                i = 0
                for light in self.directional_lights.values():
                    directional_lights[i] = create_directional_light(light, False)
                    i += 1
                i = 0
                for light in self.omni_lights.values():
                    omni_lights[i] = create_omni_light(light, False)
                    i += 1
                self.local_data['inventory_responses'].put(inventory)
        elif len(self.mesh_request_set) > 0:
            mesh_name = self.mesh_request_set.pop()
            if mesh_name in self.optix_objects:
                mesh = cortex.Mesh.new_message()
                fill_mesh(mesh, self.optix_objects[mesh_name])
                self.local_data['mesh_responses'].put(mesh)
            else:
                logger.error('Mesh ' + mesh_name + ' does not exist')
        else:
            for texture_type, texture_request_set in self.texture_request_sets.items():
                if texture_request_set:
                    texture_identifier = texture_request_set.pop()
                    if texture_identifier in self.optix_textures:
                        texture = cortex.Texture.new_message()
                        fill_texture(texture, self.optix_textures[texture_identifier])
                        if texture_type not in self.local_data['texture_responses']:
                            self.local_data['texture_responses'][texture_type] = Queue() 
                        self.local_data['texture_responses'][texture_type].put(texture)
                    else:
                        logger.error('Texture ' + texture_identifier + ' does not exist')
                    break # after one texture

            for texture_type, uvs_request_set in self.uvs_request_sets.items():
                if uvs_request_set:
                    uvs_identifier = uvs_request_set.pop()
                    if uvs_identifier in self.optix_objects:
                        uvs = cortex.Uvs.new_message()
                        fill_uvs(uvs, self.optix_objects[uvs_identifier])
                        if texture_type not in self.local_data['uvs_responses']:
                            self.local_data['uvs_responses'][texture_type] = Queue() 
                        self.local_data['uvs_responses'][texture_type].put(uvs)
                    else:
                        logger.error('Uvs ' + uvs_identifier + ' does not exist')
                    break # after one uvs

        # Create and fill out inventory message
        if self.send_json:
            inventory = {
                'instances': [],
                'lights': {
                    'ambient_lights': [],
                    'directional_lights': [],
                    'omni_lights': [],
                    'spotlight_lights': []
                }
            }
            instances = inventory['instances']
            for bge_obj in self.dynamic_instances:
                instances.append(create_instance_msg(bge_obj, bpy_objs[bge_obj.name], True))
            self.local_data['inventory_updates'] = inventory
        else:
            inventory = cortex.Inventory.new_message()
            instances = inventory.init('instances', len(self.dynamic_instances))
            lights = inventory.lights
            ambient_lights = lights.init('ambientLights', 0)
            directional_lights = lights.init('directionalLights', 0)
            omni_lights = lights.init('omniLights', 0)
            spotlight_lights = lights.init('spotlightLights', 0)
            i = 0
            for bge_obj in self.dynamic_instances:
                instances[i] = create_instance_msg(bge_obj, bpy_objs[bge_obj.name], False)
                i += 1
            self.local_data['inventory_updates'] = inventory
        