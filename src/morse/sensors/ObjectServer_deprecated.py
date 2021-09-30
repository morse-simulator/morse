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
import time
from . import Object_deprecated_capnp as Object_capnp

__author__     = "David Battle"
__copyright__  = "Copyright 2017, Mission Systems Pty Ltd"
__license__    = "BSD"
__version__    = "1.0.9"
__maintainer__ = "David Battle"
__email__      = "david.battle@missionsystems.com.au"
__status__     = "Production"

#########################################################################
# A simple object server for Morse. Currently it only accepts two verbs #
# (get) which either returns data corresponding to a specific object    #
# (noun), or just returns a nested inventory if no noun is given.       #
# (reset) resets the local cache of data blocks that have been sent.    #
# We might think of some other verbs later on.                          #
#########################################################################

flatten = lambda l: [item for sublist in l for item in sublist]

# Set logger level - DEBUG used for timing of all messages sent
logger.setLevel(logging.INFO)
LOG_QUEUE_INTERVAL_SIZE = 100

def get_pose(obj):

    position = obj.worldPosition
    orientation = obj.worldOrientation.to_euler()
    pose = {obj.name: [list(position), list(orientation)]}

    return pose

def triangulate_object(obj):

    me = obj.data
    # Get a BMesh representation
    bm = bmesh.new()
    bm.from_mesh(me)

    bmesh.ops.triangulate(bm, faces=bm.faces[:], quad_method=0, ngon_method=0)

    # Finish up, write the bmesh back to the mesh
    bm.to_mesh(me)
    bm.free()

def get_data_block(obj):

    # New faster code for object export
    vertices = flatten([list(v.co) for v in obj.data.vertices])
    faces = flatten([list(f.vertices) for f in obj.data.polygons])

    # Default mesh colour
    default = (1.0, 1.0, 1.0)

    data_block = {
       'Vertices' : vertices,
       'Faces'    : faces,
       'Colour'   : default
    }

    # Check for materials
    slots = obj.material_slots

    if len(slots):

        # Get the first material
        mat = slots[0].material

        # Get the basic mesh colour as an RGB tuple,
        data_block['Colour'] = mat.diffuse_color[:]

        # Get game properties
        props = obj.game.properties

        # Do we need a texture?
        if 'texture' in props:
            if props['texture'].value:

                # Get first material
                mat = obj.material_slots[0].material

                # Get first texture image
                im = mat.texture_slots[0].texture.image

                # Image dimensions
                data_block['Dims'] = im.size[:]

                # Append texture values [0:1]
                data_block['Texture'] = im.pixels[:]

                # Per-vertex UV coordinates
                uvs = np.zeros((len(vertices)//3,2))
                uv_layer = obj.data.uv_layers.active.data

                # Loop over loops
                for loop in obj.data.loops:
                    uvs[loop.vertex_index,:] = uv_layer[loop.index].uv

                # Convert numpy array to flat list
                data_block['UVs'] = uvs.flatten().tolist()

    return data_block

# The easiest way to create the Cap'n Proto structures is to create them fully
# as plain python objects then assign them once. This is easier (and safer) than
# using things like init_resizable_list().
def fill_data_block( data_block, obj ):

    # New faster code for object export
    vertices = flatten([list(v.co) for v in obj.data.vertices])
    faces = flatten([list(f.vertices) for f in obj.data.polygons])

    mesh_colour = ( 1.0, 1.0, 1.0 )

    # Check for materials
    slots = obj.material_slots 

    # Add faces and vertices
    data_block.mesh.vertices = vertices
    data_block.mesh.faces = faces;

    if len( slots ):
        # Get the first material
        mat = slots[0].material
        # Get the basic mesh colour as an RGB tuple,
        mesh_colour = mat.diffuse_color[:]
        data_block.meshColour = mesh_colour
        # Get game properties
        props = obj.game.properties
        # Do we need texture?
        if 'texture' in props:
            if props['texture'].value:
                logger.debug( "adding texture" )

                # Get first texture image
                im = mat.texture_slots[0].texture.image

                # Image dimensions
                data_block.dims = im.size[:]

                # Append RGBA texture
                if( im.channels != 4 ): logger.warning( 'expected four channel images: got %s' % im.channels )

                data_block.texture = im.pixels[:]

                # Per-vertex UV coordinates
                uvs = np.zeros((len(vertices)//3,2))
                uv_layer = obj.data.uv_layers.active.data

                # Loop over loops
                for loop in obj.data.loops:
                    uvs[loop.vertex_index,:] = uv_layer[loop.index].uv

                # Convert numpy array to flat list
                data_block.uvs = uvs.flatten().tolist() # uvs.tolist() not provably faster

class ObjectserverDeprecated(morse.core.sensor.Sensor):

    _name = 'Objectserver'
    _short_desc = 'MOOS server to publish serialised object data'

    add_data('scene_query',[],'list','Queue for query strings')
    add_data('scene_data' ,'','string','Scene data output in JSON format')
    add_data('object_data_binary', '', 'binary', 'Object data in binary format')
    add_data('test_data', '', 'binary', 'test binary data')

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
        remove_items = ['Scene_Script_Holder','CameraFP',
                        '__default__cam__','MORSE.Properties',
                        '__morse_dt_analyser']

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

        self.prev_queue_size = 0

    def default_action(self):

        # Get the potential scene query
        queue = self.local_data['scene_query']

        # Get every object in the scene
        objects = bpymorse.get_objects()

        # Are there any queries?
        if len(queue):

            # Get query verb and noun
            query = queue.pop(0).split()

            if query:

                # Look for a verb
                if query[0] == 'get':

                    # Is there a noun?
                    if len(query) > 1:

                        # Log queue size increases and at intervals which processing
                        queue_size = len(queue)
                        if queue_size > self.prev_queue_size or queue_size % LOG_QUEUE_INTERVAL_SIZE == 0:
                            logger.info('Message queue length: ' + str(queue_size))
                        self.prev_queue_size = queue_size

                        # Yes: the noun is the object name
                        obj_name = query[1]

                        # Check it exists
                        if obj_name in objects:

                            # Prepare a string for later logging
                            log_string = 'Processing Object \"' + obj_name + '\" '

                            # Initialise the timer
                            start_time = time.time()

                            obj = objects[obj_name]
                            data_name = obj.data.name
                            obj_scale = (obj.scale.x,
                                         obj.scale.y,
                                         obj.scale.z)

                            # Get game properties
                            props = obj.game.properties

                            # Always send as binary capnp
                            send_as_binary = data_name not in self.data_blocks_sent

                            if send_as_binary:
                                # Send as binary capnp
                                log_string += ' as a capnp binary message...'
                                logger.debug(log_string)

                                # Create new capnp message
                                sim_object = Object_capnp.Object.new_message()
                                sim_object.objName = obj_name
                                sim_object.dataName = data_name
                                sim_object.scaleX = obj_scale[0]
                                sim_object.scaleY = obj_scale[1]
                                sim_object.scaleZ = obj_scale[2]
                                # Default to lossless water material if not specified
                                sim_object.rho = 1000
                                sim_object.speed = 1500
                                sim_object.alpha = 0
                                if 'rho' in props: sim_object.rho = props['rho'].value
                                if 'speed' in props: sim_object.speed = props['speed'].value
                                if 'alpha' in props: sim_object.alpha = props['alpha'].value
                                # defaults - see plidarsim/BlenderObject.cpp
                                sim_object.kd = 1.0
                                sim_object.ks = 0.0
                                if 'Kd' in props: sim_object.kd = props['Kd'].value
                                if 'Ks' in props: sim_object.ks = props['Ks'].value

                                # Track progress
                                deltat_partial = time.time() - start_time
                                logger.debug('\t... created blender object in ' + str(deltat_partial) + ' s ...')

                                # Is this a new data block?
                                if not data_name in self.data_blocks_sent:
                                    fill_data_block( sim_object.dataBlock, obj )
                                    self.data_blocks_sent.append(data_name)

                                    # Track progress
                                    deltat_partial_2 = time.time() - start_time - deltat_partial
                                    logger.debug('\t... filled data block in ' + str(deltat_partial_2) + ' s ...')
                                else:
                                    deltat_partial_2 = 0
                                    logger.debug('\t... ' + data_name + ' data block has already been filled ...')

                                self.local_data['object_data_binary'] = sim_object.to_bytes()
                                msg_len = len( self.local_data['object_data_binary'] )

                                # Track progress
                                deltat_partial_3 = time.time() - start_time - deltat_partial - deltat_partial_2
                                logger.debug('\t... converted to bytes and length calculated in ' + str(deltat_partial_3) + ' s ...')
                            else:
                                # Send as JSON
                                log_string += ' as a json string message...'
                                logger.debug(log_string);

                                data = {
                                   'Obj_name'  : obj_name,
                                   'Obj_scale' : obj_scale,
                                   'Data_name' : data_name
                                }

                                # Append game properties in
                                # lower case to avoid confusion
                                for p in props:
                                    data[p.name.lower()] = p.value

                                # Track progress
                                deltat_partial = time.time() - start_time
                                logger.debug('\t... created json struct in ' + str(deltat_partial) + ' s ...')

                                # Is this a new data block?
                                if not data_name in self.data_blocks_sent:

                                    # A data block will only be attached
                                    # to an object if it has a single user.
                                    data['Data_block'] = get_data_block(obj)
                                    self.data_blocks_sent.append(data_name)

                                    # Track progress
                                    deltat_partial_2 = time.time() - start_time - deltat_partial
                                    logger.debug('\t... obtained data block in ' + str(deltat_partial_2) + ' s ...')
                                else:
                                    deltat_partial_2 = 0
                                    logger.debug('\t... ' + data_name + ' data block has already been filled ...')

                                # Publish the encoded object data
                                self.local_data['scene_data'] = json.dumps(data)
                                msg_len = len( self.local_data['scene_data'] )

                                # Track progress
                                deltat_partial_3 = time.time() - start_time - deltat_partial - deltat_partial_2
                                logger.debug('\t... converted to string and length calculated in ' + str(deltat_partial_3) + ' s ...')

                            # How long did the encoding take?
                            deltat = time.time() - start_time

                            logger.debug('\t... object converted in total time ' + str(deltat) + ' s.')
                        else:
                            print('Error: unrecognized object (%s).' % obj_name)
                    else:

                        # We only update dynamic objects
                        # and objects which have stabilised.
                        # Objects which have stabilised are
                        # then removed from the inventory.
                        # Ultimately, there will only be
                        # dynamic objects in the inventory.

                        obj_list = {}

                        # Scan through object list
                        for obj in self.Optix_objects:

                            # Check for dynamic object
                            if obj.get('dynamic', False):

                                # Update dynamic object
                                obj_list.update(get_pose(obj))

                            # Update object if non-physics or static
                            # Objects of this type only get updated once
                            elif obj.getPhysicsId() == 0 or obj.linearVelocity.length == 0:

                                # Yes - get the latest pose
                                obj_list.update(get_pose(obj))

                                # Now remove from the list
                                self.Optix_objects.remove(obj)

                        if len(obj_list):

                            # Compose update message
                            message = {'Inventory' : obj_list}

                            # Publish the inventory update
                            self.local_data['scene_data'] = json.dumps(message)

                elif query[0] == 'reset':

                    # New connection - reset cache
                    self.data_blocks_sent = []
                    print('Data block cache reset.')

                    obj_list = {}

                    # Generate a full inventory
                    for obj in self.Optix_objects:
                        obj_list.update(get_pose(obj))

                    # Compose inventory message
                    message = {'Inventory' : obj_list}

                    # Publish object inventory
                    self.local_data['scene_data'] = json.dumps(message)

                    print('Inventory sent.')

                else:
                    print('Error: unrecognised query (%s).' % query[0])
