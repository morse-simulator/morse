import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.sensor
from morse.core.services import service, async_service
from morse.core import status, blenderapi
from morse.helpers.components import add_data, add_property
from morse.builder import bpymorse
import numpy as np
import bmesh
import bpy
import capnp
import json
from queue import Queue
import time

import sys
sys.path.append("/usr/local/etc/cortex")
import cortex_capnp as cortex

flatten = lambda l: [item for sublist in l for item in sublist]

# Set logger level - DEBUG used for timing of all messages sent
logger.setLevel(logging.INFO)

def fill_instance(instance, bge_obj, bpy_obj):
    instance.instanceName   = bge_obj.name
    instance.objectName   = bpy_obj.data.name
    position = bge_obj.worldPosition
    instance.position.x     = position.x
    instance.position.y     = position.y
    instance.position.z     = position.z
    instance.scale.x        = bpy_obj.scale.x
    instance.scale.y        = bpy_obj.scale.y
    instance.scale.z        = bpy_obj.scale.z
    orientation = bge_obj.worldOrientation.to_euler() # TODO should this be quaternion?
    instance.orientation.x  = orientation.x
    instance.orientation.y  = orientation.y
    instance.orientation.z  = orientation.z
    try:
        linear_velocity = bge_obj.worldLinearVelocity
        instance.velocity.x     = linear_velocity.x
        instance.velocity.y     = linear_velocity.y
        instance.velocity.z     = linear_velocity.z
        angular_velocity = bge_obj.worldAngularVelocity
        # instance.angular_velocity.x = angular_velocity.x
        # instance.angular_velocity.y = angular_velocity.y
        # instance.angular_velocity.z = angular_velocity.z
    except: pass
    properties = bpy_obj.game.properties
    if 'Kd' in properties: instance.reflectance.kd = properties['Kd'].value
    if 'Ks' in properties: instance.reflectance.ks = properties['Ks'].value

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

def fill_mesh(mesh, bpy_obj):

    # New faster code for object export
    # vertices = flatten([list(v.co) for v in obj.data.vertices])
    # faces = flatten([list(f.vertices) for f in obj.data.polygons])
    # vertices_msg = mesh.init('vertices', len(vertices))
    # vertices_msg[:] = vertices
    # faces_msg = mesh.init('vertices', len(vertices))
    # faces_msg[:] = faces

    # Fill mesh
    mesh.objectName = bpy_obj.data.name
    mesh.vertices = flatten([list(v.co) for v in bpy_obj.data.vertices])
    mesh.faces = flatten([list(f.vertices) for f in bpy_obj.data.polygons])
    mesh.textureDims = [0, 0]

    # Check for materials
    slots = bpy_obj.material_slots

    if len(slots):
        # Get game properties
        props = bpy_obj.game.properties
        # Check for texture
        if 'texture' in props and props['texture'].value:
            # Get the first material
            mat = slots[0].material

            # Get first texture image
            im = mat.texture_slots[0].texture.image

            # Image dimensions
            mesh.textureDims = im.size[:]

            # Append RGBA texture
            if (im.channels != 4):
                logger.warning( 'expected four channel images: got %s' % im.channels )

            mesh.texture = im.pixels[:]

            # Per-vertex UV coordinates
            uvs = np.zeros((len(mesh.vertices)//3,2))
            uv_layer = bpy_obj.data.uv_layers.active.data

            # Loop over loops
            for loop in bpy_obj.data.loops:
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

        # List of data blocks transferred
        # Object instances will re-use these...
        self.data_blocks_sent = []

        # Special Morse items to remove from inventory
        remove_items = ['Scene_Script_Holder','CameraFP', '__default__cam__','MORSE.Properties', '__morse_dt_analyser']

        self.Dynamic_objects = []

        # Get dynamic objects
        for obj in bpy_objs:
            for prop in obj.game.properties:
                if 'dynamic' in prop.name.lower():
                    if prop.value == True:
                         self.Dynamic_objects.append(bge_objs[obj.name])

        # Not included in Optix updates
        for obj in bpy_objs:
            for prop in obj.game.properties:
                if 'optix' in prop.name.lower():
                    if prop.value == False:
                        remove_items.append(obj.name)

        # Get visible top levelers
        self.Optix_objects = [obj for obj in bge_objs
                             if obj.parent is None
                             and not obj.name in remove_items
                             and obj.meshes and obj.visible]

        # Included in Optix updates
        for obj in bpy_objs:
            for prop in obj.game.properties:
                if 'optix' in prop.name.lower():
                    if prop.value:
                        self.Optix_objects.append(bge_objs[obj.name])

        # Check objects for triangulation
        for bge_obj in self.Optix_objects:
            bpy_obj = bpy_objs[bge_obj.name]
            triangulated = True
            for p in bpy_obj.data.polygons:
                if len(p.vertices) > 3:
                    triangulated = False

            if not triangulated:
                logger.warning('WARNING: Object %s must be triangulated...' % (bpy_obj.name))
                triangulate_object(bpy_obj)

        logger.info('Found %d dynamic objects in scene' % len(self.Dynamic_objects))
        logger.info('Component initialized, runs at %.2f Hz', self.frequency)

        self.prev_queue_size = 0 # TODO delete

        self.object_request_dict = {}
        self.object_request_queue = Queue()

    def default_action(self):

        # Convert object request queue to queue+map data structure (for efficient impl)
        while not self.local_data['object_requests'].empty():
            # Read the object request
            object_request = cortex.ObjectRequest.read(self.local_data['object_requests'].get())
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
            instances = inventory.inventory.init('instances', len(self.Optix_objects))
            for i in range(len(self.Optix_objects)):
                fill_instance(instances[i], self.Optix_objects[i], bpy_objs[self.Optix_objects[i].name])
                
            # Add the inventory to the responses
            self.local_data['inventory_responses'].put(inventory)
        elif not self.object_request_queue.empty():
            # Obtain object request data
            object_name = self.object_request_queue.get()
            data_types = self.object_request_dict.pop(object_name)

            if object_name in self.Optix_objects:
                # Iterate through data types, create each, and push onto object responses queue
                for data_type in data_types:
                    if data_type == cortex.ObjectRequest.DataType.mesh:
                        mesh = cortex.Mesh.new_message()
                        fill_mesh(mesh, self.Optix_objects[object_name], bpy_objs[object_name])
                        self.local_data['object_responses'].put(mesh)
                    else:
                        logger.error('Unknown data type for object ' + object_name)
            else:
                logger.error('Object ' + object_name + ' does not exist')


        # Scan through object list and add dynamic objects to update list
        obj_update_list = []
        for obj in self.Optix_objects:
            if obj.get('dynamic', False):
                obj_update_list.append(obj)

        if len(obj_update_list) > 0:
            # Create and fill out inventory message
            inventory = cortex.Inventory.new_message()
            instances = inventory.init('instances', len(obj_update_list))
            for i in range(len(obj_update_list)):
                fill_instance(instances[i], obj_update_list[i], bpy_objs[obj_update_list[i].name])
            self.local_data['inventory_updates'] = inventory