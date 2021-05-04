import logging; logger = logging.getLogger("morse." + __name__)
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

import sys
sys.path.extend(["/usr/local/share/", "/usr/local/share/cortex"])
import cortex_capnp as cortex

flatten = lambda l: [item for sublist in l for item in sublist]

# Set logger level - DEBUG used for timing of all messages sent
logger.setLevel(logging.INFO)

def create_trigger_msg(position, rotation, dim_x, dim_y, dim_z, dict_msg = True):
    if not isinstance(rotation, Quaternion):
        rotation = rotation.to_quaternion()
    if dict_msg:
        launch_trigger = {
           'pose': {
                'position' : {'x': position.x, 'y': position.y, 'z': position.z},
                'rotation' : {'x': rotation.x, 'y': rotation.y, 'z': rotation.z, 'w': rotation.w}
           },
           'dimensions'    : {'x': dim_x,      'y': dim_y,      'z': dim_z}
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

    if dict_msg:
        # Core properties
        instance = {
            'instanceName' : optix_instance.name,
            'properties' : [{'identifier': optix_object.data.name, 'propertyType': 'MESH'}],
            'transform': {
                'position' : {'x': position.x, 'y': position.y, 'z': position.z},
                'rotation' : {'x': rotation.x, 'y': rotation.y, 'z': rotation.z, 'w': rotation.w},
                'scale' :    {'x': scale.x,    'y': scale.y,    'z': scale.z}
            }
        }

        # Extra properties
        try:
            linear_velocity = optix_instance.worldLinearVelocity
            lin_vel = {}
            lin_vel['x']  = linear_velocity.x
            lin_vel['y']  = linear_velocity.y
            lin_vel['z']  = linear_velocity.z
            instance['linearVelocity'] = lin_vel
        except: pass
        try:
            angular_velocity = optix_instance.worldAngularVelocity
            ang_vel = {}
            ang_vel['x']  = angular_velocity.x
            ang_vel['y']  = angular_velocity.y
            ang_vel['z']  = angular_velocity.z
            instance['angularVelocity'] = ang_vel
        except: pass
        instance['reflectance'] = {}
        if 'Kd' in properties:
            instance['reflectance']['kd'] = properties['Kd'].value
        else:
            instance['reflectance']['kd'] = 1.0
        if 'Ks' in properties:
            instance['reflectance']['ks'] = properties['Ks'].value
        else:
            instance['reflectance']['ks'] = 0.0
        return instance
    else:
        instance = cortex.Instance.new_message()

        # Core properties
        instance.instanceName              = optix_instance.name
        instance_props                     = instance.init('properties', 1)
        instance_props[0].identifier       = optix_object.data.name
        instance_props[0].propertyType     = cortex.ObjectProperty.PropertyType.mesh
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
            linear_velocity = optix_instance.worldLinearVelocity
            instance.linearVelocity.x  = linear_velocity.x
            instance.linearVelocity.y  = linear_velocity.y
            instance.linearVelocity.z  = linear_velocity.z
        except: pass
        try:
            angular_velocity = optix_instance.worldAngularVelocity
            instance.angularVelocity.x = angular_velocity.x
            instance.angularVelocity.y = angular_velocity.y
            instance.angularVelocity.z = angular_velocity.z
        except: pass
        if 'Kd' in properties:
            instance.reflectance.kd = properties['Kd'].value
        else:
            instance.reflectance.kd = 1.0
        if 'Ks' in properties:
            instance.reflectance.ks = properties['Ks'].value
        else:
            instance.reflectance.ks = 0.0
        return instance

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

    # New faster code for object export
    # vertices = flatten([list(v.co) for v in obj.data.vertices])
    # faces = flatten([list(f.vertices) for f in obj.data.polygons])
    # vertices_msg = mesh.init('vertices', len(vertices))
    # vertices_msg[:] = vertices
    # faces_msg = mesh.init('vertices', len(vertices))
    # faces_msg[:] = faces

    # Fill mesh
    mesh.identifier = optix_obj.data.name
    mesh.vertices = flatten([list(v.co) for v in optix_obj.data.vertices])
    mesh.faces = flatten([list(f.vertices) for f in optix_obj.data.polygons])
    mesh.textureDims.x = 0
    mesh.textureDims.y = 0

    # Check for materials
    slots = optix_obj.material_slots

    if len(slots):
        # Get game properties
        props = optix_obj.game.properties
        # Check for texture
        if 'texture' in props and props['texture'].value:
            # Get the first material
            mat = slots[0].material

            # Get first texture image
            im = mat.texture_slots[0].texture.image

            # Image dimensions
            mesh.textureDims.x = im.size[0]
            mesh.textureDims.y = im.size[1]

            # Append RGBA texture
            if (im.channels != 4):
                logger.warning( 'expected four channel images: got %s' % im.channels )

            mesh.texture = im.pixels[:]

            # Per-vertex UV coordinates
            uvs = np.zeros((len(mesh.vertices)//3,2))
            uv_layer = optix_obj.data.uv_layers.active.data

            # Loop over loops
            for loop in optix_obj.data.loops:
                uvs[loop.vertex_index,:] = uv_layer[loop.index].uv

            # Convert numpy array to flat list
            mesh.uvs = uvs.flatten().tolist() # uvs.tolist() not provably faster

class Objectserver(morse.core.sensor.Sensor):

    _name = 'Objectserver'
    _short_desc = 'MOOS server to publish serialised object data'

    # add_data('scene_query',[],'list','Queue for query strings')
    add_data('inventory_requests', Queue(), 'queue', 'Queue for inventory requests')
    add_data('inventory_responses', Queue(), 'queue', 'Queue for inventory responses')
    add_data('inventory_updates', None, 'inventory', 'Inventory for inventory updates')
    add_data('object_requests', Queue(), 'queue', 'Queue for object requests')
    add_data('object_responses', Queue(), 'queue', 'Queue for object responses')

    add_property('send_json', True, 'send_json',  'bool', 'Send small messages as json')

    def __init__(self, obj, parent=None):

        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)

        scene = blenderapi.scene()

        # All bge objects in scene
        bge_objs = scene.objects

        # All bpy objects in scene
        bpy_objs = bpymorse.get_objects()

        # Data structures for optix
        self.dynamic_instances = []
        self.optix_instances = []
        self.optix_objects = {}
        self.optix_ignored_instances = []
        for obj in bpy_objs:
            props = obj.game.properties
            if 'optix' in props and props['optix'].value:
                self.optix_instances.append(bge_objs[obj.name])
                if not obj.data.name in self.optix_objects:
                    self.optix_objects[obj.data.name] = obj
                if 'dynamic' in props and props['dynamic'].value:
                    self.dynamic_instances.append(bge_objs[obj.name])
            else:
                self.optix_ignored_instances.append(obj.name)
        logger.info("Dynamic Instances: " + str(self.dynamic_instances))
        logger.info("Optix Instances: " + str(self.optix_instances))
        logger.info("Optix Objects: " + str(self.optix_objects))
        logger.info("Optix Ignored Instances: " + str(self.optix_ignored_instances))

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

        self.prev_queue_size = 0 # TODO delete

        # self.object_request_dict = {}
        # self.object_request_queue = Queue()
        self.object_request_set = set() # TODO: change for ordered set

    def default_action(self):

        # Convert object request queue to queue+map data structure (for efficient impl)
        while not self.local_data['object_requests'].empty():
            # Read the object request
            object_request = cortex.ObjectRequest.from_bytes(self.local_data['object_requests'].get())
            for prop in object_request.properties:
                self.object_request_set.add((prop.identifier, prop.propertyType))
            # # Process request efficiently
            # if not object_request.objectName in self.object_request_dict:
            #     self.object_request_dict[object_request.objectName] = set()
            #     self.object_request_queue.put(object_request.objectName)
            # for data_type in object_request.dataTypes:
            #     # Add to set (duplicates automatically removed)
            #     self.object_request_dict[object_request.objectName].add(data_type)

        bpy_objs = bpymorse.get_objects()

        if not self.local_data['inventory_requests'].empty():
            pid = self.local_data['inventory_requests'].get()
            if self.send_json:
                inventory = {'inventory': {'instances': []}, 'uid': pid}
                instances = inventory['inventory']['instances']

                # Add the instances
                for i in range(len(self.optix_instances)):
                    instances.append(create_instance_msg(
                        self.optix_instances[i], bpy_objs[self.optix_instances[i].name], True))
                self.local_data['inventory_responses'].put(inventory)
            else:
                # Create capnp unique inventory
                inventory = cortex.UniqueInventory.new_message()
                inventory.uid = pid
                instances = inventory.inventory.init('instances', len(self.optix_instances))
                for i in range(len(self.optix_instances)):
                    instances[i] = create_instance_msg(self.optix_instances[i], bpy_objs[self.optix_instances[i].name], False)
                self.local_data['inventory_responses'].put(inventory)
        elif len(self.object_request_set) > 0:
            # Obtain object request data
            prop = self.object_request_set.pop()
            identifier = prop[0]
            prop_type = prop[1]

            if identifier in self.optix_objects:
                # Iterate through data types, create each, and push onto object responses queue
                if prop_type == cortex.ObjectProperty.PropertyType.mesh:
                    mesh = cortex.Mesh.new_message()
                    fill_mesh(mesh, self.optix_objects[identifier])
                    self.local_data['object_responses'].put(mesh)
                else:
                    logger.error('Unhandled property type for identifier ' + identifier)
            else:
                logger.error('Identifier ' + identifier + ' does not exist')

        # Create and fill out inventory message
        if self.send_json:
            inventory = {'instances': []}
            instances = inventory['instances']
            for bge_obj in self.dynamic_instances:
                instances.append(create_instance_msg(bge_obj, bpy_objs[bge_obj.name], True))
            self.local_data['inventory_updates'] = inventory
        else:
            inventory = cortex.Inventory.new_message()
            instances = inventory.init('instances', len(self.dynamic_instances))
            i = 0
            for bge_obj in self.dynamic_instances:
                instances[i] = create_instance_msg(bge_obj, bpy_objs[bge_obj.name], False)
                i += 1
            self.local_data['inventory_updates'] = inventory
        