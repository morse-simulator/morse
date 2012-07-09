import logging; logger = logging.getLogger("morse." + __name__)
import math
import time
import re
import bge
from morse.middleware.pocolibs.sensors.Viman_Poster import ors_viman_poster
from morse.helpers.transformation import Transformation3d
from morse.helpers import passive_objects

object_config_file = "objectList_cfg"

ABSOLUTE = 0
RELATIVE = 1

#logger.setLevel(logging.DEBUG)

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """

    component_name = component_instance.blender_obj.name
    parent_name = component_instance.robot_parent.blender_obj.name
    # Check if the name of the poster has been given in mw_data
    try:
        # It should be the 4th parameter
        poster_name = mw_data[3]
    except IndexError as detail:
        # Compose the name of the poster, based on the parent and module names
        poster_name = 'viman_{0}_{1}'.format(parent_name, component_name)

    poster_id = init_viman_poster(self, component_instance, poster_name)
    if poster_id != None:
        logger.info("Pocolibs created poster of type viman")
        component_instance.output_functions.append(function)
        # Store the name of the port
        self._poster_dict[component_name] = poster_id


def init_viman_poster(self, component_instance, poster_name):
    """ Prepare the data for a viman poster """

    self.scene_object_list = []

    #If provided, read the list of ARToolkit tags to fill the list of objects
    #to export.
    self.scene_object_list += _read_object_list()

    #Complete the list with the objects already tracked by the semantic cam.
    if hasattr(bge.logic, 'passiveObjectsDict'):
        self.scene_object_list += [obj['label'] for obj in bge.logic.passiveObjectsDict.values()]

    if not self.scene_object_list:
        logger.error("No VIMAN object to track. Make sure some objects have " +\
               "the game property 'Object' defined. Disabling poster for now.")
        return None

    self.viman_data = ors_viman_poster.generate_viman_struct()

    # Init the data structures used by this poster
    self.viman_data.nbObjects = len(self.scene_object_list)
    i = 0
    for object in self.scene_object_list:
        logger.info("Adding " + object + " to the objects tracked by VIMAN")
        ors_viman_poster.set_name(self.viman_data, i, str(object))
        #self.viman_data.objects[i].name = object
        i = i + 1

    poster_id, ok = ors_viman_poster.init_data(poster_name)
    if ok == 0:
        logger.error("Creating poster. This module may not work")
        return None

    logger.info("VIMAN Poster '%s' created (ID: %d)" % (poster_name, poster_id))

    return poster_id


def write_viman(self, component_instance):
    """ Write the objects list to a poster """
    # Get the id of the poster already created
    poster_id = self._poster_dict[component_instance.blender_obj.name]
    parent = component_instance.robot_parent

    scene = bge.logic.getCurrentScene()
    
    seen_objects = [obj['name'] for obj in component_instance.local_data['visible_objects']]

    i = 0
    for object_id in self.scene_object_list:

        try:
            t = time.time()
            tacq_sec = int(t)
            tacq_usec = int((t - tacq_sec) * 1000)
            ors_viman_poster.set_tacq(self.viman_data, i, tacq_sec, tacq_usec)
            
            if object_id in seen_objects:

                object = passive_objects.obj_from_label(object_id)

                position_3d = Transformation3d(object)
                logger.debug("VIMAN " + object_id + "(" + object.name + ") is visible at " + str(position_3d))
                ors_viman_poster.set_visible(self.viman_data, i, 1)
                _fill_world_matrix(self.viman_data, position_3d, i)
                _fill_robot_matrix(self.viman_data, parent, position_3d, i)
            else:
                ors_viman_poster.set_visible (self.viman_data, i, 0)
        
        
            # Write to the poster with the data for all objects
            posted = ors_viman_poster.real_post_viman_poster(poster_id, self.viman_data)
        except KeyError as detail:
            logger.debug("WARNING: Object %s not found in the scene" % detail)
            pass
            posted = False

        i = i + 1


def _fill_world_matrix(viman_data, t3d, index):
    """ Fill the world matix part of the structure

    This function receives the Blender rotation matrix and position of an object
    It calls a module function to fill out data structure of type VimanThetaMat
    """

    nx = t3d.matrix[0][0]
    ox = t3d.matrix[1][0]
    ax = t3d.matrix[2][0]
    px = t3d.x

    ny = t3d.matrix[0][1]
    oy = t3d.matrix[1][1]
    ay = t3d.matrix[2][1]
    py = t3d.y

    nz = t3d.matrix[0][2]
    oz = t3d.matrix[1][2]
    az = t3d.matrix[2][2]
    pz = t3d.z

    result = ors_viman_poster.write_matrix (viman_data, index, ABSOLUTE,
        nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz)

def _fill_robot_matrix(viman_data, robot, obj_3dpose, index):
    """ Fill the world matix part of the structure

    This function receives the Blender rotation matrix and position of an object
    It calls a module function to fill out data structure of type VimanThetaMat
    """
    robot3d = robot.position_3d

    t3d = robot3d.transformation3d_with(obj_3dpose)

    nx = t3d.matrix[0][0]
    ox = t3d.matrix[1][0]
    ax = t3d.matrix[2][0]
    px = t3d.x

    ny = t3d.matrix[0][1]
    oy = t3d.matrix[1][1]
    ay = t3d.matrix[2][1]
    py = t3d.y

    nz = t3d.matrix[0][2]
    oz = t3d.matrix[1][2]
    az = t3d.matrix[2][2]
    pz = t3d.z

    result = ors_viman_poster.write_matrix (viman_data, index, RELATIVE,
        nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz)


def _read_object_list():
    """ Open the file specified in the object_config_file variable

    Read the list of component names"""

    scene_object_list = []
    logger.info("Reading '%s' config file for VIMAN poster" % object_config_file)

    try:
        fp = open(object_config_file, "r")
        for line in fp:
            match = re.search('object (\w+)', line)
            if match != None:
                scene_object_list.append(match.group(1))
                logger.debug("\t- %s" % match.group(1))
        fp.close()
    except IOError as detail:
        logger.debug(detail)
        logger.info("ARToolkit tag library not found. Skipping it.")
        return []

    return scene_object_list
