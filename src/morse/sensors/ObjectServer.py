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

# Create a 
def create_trigger_msg(position, rotation, dim_x, dim_y, dim_z, json = True):
    if not isinstance(rotation, Quaternion):
        rotation = rotation.to_quaternion()
    if json:
        launch_trigger = json.dumps({
           'position'       : {'x': position.x, 'y': position.y, 'z': position.z},
           'rotation'       : {'x': rotation.x, 'y': rotation.y, 'z': rotation.z, 'w': rotation.w},
           'dimensions'     : {'x': dim_x,      'y': dim_y,      'z': dim_z}
        })
    else:
        launch_trigger = cortex.LaunchTrigger.new_message()
        launch_trigger.pose.position.x = position.x
        launch_trigger.pose.position.y = position.y
        launch_trigger.pose.position.z = position.z
        launch_trigger.pose.orientation.w = rotation.w
        launch_trigger.pose.orientation.x = rotation.x
        launch_trigger.pose.orientation.y = rotation.y
        launch_trigger.pose.orientation.z = rotation.z
        launch_trigger.dimensions.x = dim_x
        launch_trigger.dimensions.y = dim_y
        launch_trigger.dimensions.z = dim_z
    return launch_trigger

def fill_instance(instance, optix_instance, optix_object):
    position = optix_instance.worldPosition
    orientation = optix_instance.worldOrientation
    scale = optix_object.scale

    instance.instanceName              = optix_instance.name
    instance.objectName                = optix_object.data.name
    instance.transform.position.x      = position.x
    instance.transform.position.y      = position.y
    instance.transform.position.z      = position.z
    instance.transform.rotation.row0.x = orientation[0][0]
    instance.transform.rotation.row0.y = orientation[0][1]
    instance.transform.rotation.row0.z = orientation[0][2]
    instance.transform.rotation.row1.x = orientation[1][0]
    instance.transform.rotation.row1.y = orientation[1][1]
    instance.transform.rotation.row1.z = orientation[1][2]
    instance.transform.rotation.row2.x = orientation[2][0]
    instance.transform.rotation.row2.y = orientation[2][1]
    instance.transform.rotation.row2.z = orientation[2][2]
    instance.transform.scale.x         = scale.x
    instance.transform.scale.y         = scale.y
    instance.transform.scale.z         = scale.z
    # orientation = optix_instance.worldOrientation.to_euler() # TODO should this be quaternion?
    # instance.orientation.x  = orientation.x
    # instance.orientation.y  = orientation.y
    # instance.orientation.z  = orientation.z

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
    properties = optix_object.game.properties
    if 'Kd' in properties:
        instance.reflectance.kd = properties['Kd'].value
    else:
        instance.reflectance.kd = 1.0
    if 'Ks' in properties:
        instance.reflectance.ks = properties['Ks'].value
    else:
        instance.reflectance.ks = 0.0

def triangulate_object(obj):

    me = obj.data
    # Get a BMesh representation
    bm = bmesh.new()
    bm.from_mesh(me)

    bmesh.ops.triangulate(bm, faces=bm.faces[:], quad_method=0, ngon_method=0)

    # Finish up, write the bmesh back to the mesh
    bm.to_mesh(me)
    bm.free()

# def get_mesh(obj):
#     # Create mesh msg instance
#     mesh = cortex.Mesh.new_message()

#     # New faster code for object export
#     flat_vertices = flatten([list(v.co) for v in obj.data.vertices])
#     flat_faces = flatten([list(f.vertices) for f in obj.data.polygons])

#     vertices = mesh.init('vertices', len(flat_vertices))
#     vertices[:] = flat_vertices
#     faces = mesh.init('faces', len(flat_faces))
#     faces[:] = flat_faces
#     texture_dims = mesh.init('textureDims', 2)
#     texture_dims[0] = 0
#     texture_dims[1] = 0

#     # Check for materials
#     slots = obj.material_slots

#     if len(slots):

#         # Get the first material
#         mat = slots[0].material

#         # Get game properties
#         props = obj.game.properties

#         # Do we need a texture?
#         if 'texture' in props:
#             if props['texture'].value:

#                 # Get first material
#                 mat = obj.material_slots[0].material

#                 # Get first texture image
#                 im = mat.texture_slots[0].texture.image

#                 # Image dimensions
#                 texture_dims[0] = im.size[0]
#                 texture_dims[1] = im.size[1]
#                 texture = mesh.init('texture', len(im.size[0]*im.size[1]))

#                 # Append texture values [0:1]
#                 texture[:] = im.pixels[:]

#                 # Per-vertex UV coordinates
#                 uvs_array = np.zeros((len(vertices)//3,2))
#                 uv_layer = obj.data.uv_layers.active.data

#                 # Loop over loops
#                 for loop in obj.data.loops:
#                     uvs_array[loop.vertex_index,:] = uv_layer[loop.index].uv

#                 # Flatten
#                 uvs_array = uvs_array.flatten().tolist()

#                 # Convert numpy array to flat list
#                 uvs = mesh.init('uvs', len(uvs_array))
#                 uvs[:] = uvs_array

#     return mesh

def fill_mesh(mesh, optix_obj):

    # New faster code for object export
    # vertices = flatten([list(v.co) for v in obj.data.vertices])
    # faces = flatten([list(f.vertices) for f in obj.data.polygons])
    # vertices_msg = mesh.init('vertices', len(vertices))
    # vertices_msg[:] = vertices
    # faces_msg = mesh.init('vertices', len(vertices))
    # faces_msg[:] = faces

    # Fill mesh
    mesh.objectName = optix_obj.data.name
    mesh.vertices = flatten([list(v.co) for v in optix_obj.data.vertices])
    mesh.faces = flatten([list(f.vertices) for f in optix_obj.data.polygons])
    mesh.textureDims.x = 0
    mesh.textureDims.y = 0
    # mesh.textureDims = [0, 0]

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
            # mesh.textureDims = im.size[:]

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
        for obj in bpy_objs:
            props = obj.game.properties
            if 'optix' in props and props['optix']:
                self.optix_instances.append(bge_objs[obj.name])
                if not obj.data.name in self.optix_objects:
                    self.optix_objects[obj.data.name] = obj
                if 'dynamic' in props and props['dynamic']:
                    self.dynamic_instances.append(bge_objs[obj.name])
        logger.info("Dynamic Instances: " + str(self.dynamic_instances))
        logger.info("Optix Instances: " + str(self.optix_instances))
        logger.info("Optix Objects: " + str(self.optix_objects))

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

        self.object_request_dict = {}
        self.object_request_queue = Queue()

    def default_action(self):

        # Convert object request queue to queue+map data structure (for efficient impl)
        while not self.local_data['object_requests'].empty():
            # Read the object request
            object_request = cortex.ObjectRequest.from_bytes(self.local_data['object_requests'].get())
            # Process request efficiently
            if not object_request.objectName in self.object_request_dict:
                self.object_request_dict[object_request.objectName] = set()
                self.object_request_queue.put(object_request.objectName)
            for data_type in object_request.dataTypes:
                # Add to set (duplicates automatically removed)
                self.object_request_dict[object_request.objectName].add(data_type)

        bpy_objs = bpymorse.get_objects()

        if not self.local_data['inventory_requests'].empty():
            # Create message with pid
            inventory = cortex.UniqueInventory.new_message()
            pid = self.local_data['inventory_requests'].get()
            inventory.uid = pid

            # Add the instances
            instances = inventory.inventory.init('instances', len(self.optix_instances))
            for i in range(len(self.optix_instances)):
                fill_instance(instances[i], self.optix_instances[i], bpy_objs[self.optix_instances[i].name])
                
            # Add the inventory to the responses
            self.local_data['inventory_responses'].put(inventory)
        elif not self.object_request_queue.empty():
            # Obtain object request data
            object_name = self.object_request_queue.get()
            data_types = self.object_request_dict.pop(object_name)

            if object_name in self.optix_objects:
                # Iterate through data types, create each, and push onto object responses queue
                for data_type in data_types:
                    if data_type == cortex.ObjectRequest.DataType.mesh:
                        mesh = cortex.Mesh.new_message()
                        fill_mesh(mesh, self.optix_objects[object_name])
                        self.local_data['object_responses'].put(mesh)
                    else:
                        logger.error('Unknown data type for object ' + object_name)
            else:
                logger.error('Object ' + object_name + ' does not exist')

        # Create and fill out inventory message
        inventory = cortex.Inventory.new_message()
        instances = inventory.init('instances', len(self.dynamic_instances))
        i = 0
        for bge_obj in self.dynamic_instances:
            fill_instance(instances[i], bge_obj, bpy_objs[bge_obj.name])
            i += 1
        self.local_data['inventory_updates'] = inventory
        