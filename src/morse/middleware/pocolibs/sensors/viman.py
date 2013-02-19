import logging; logger = logging.getLogger("morse." + __name__)
import time
import re
from morse.core import blenderapi
from morse.helpers.transformation import Transformation3d
from morse.helpers import passive_objects
from morse.middleware.pocolibs_datastream import *
from viman.struct import *

object_config_file = "objectList_cfg"

def _fill_world_matrix(obj, t3d):
    """ Fill the world matix part of the structure

    This function receives the Blender rotation matrix and position of an object
    It calls a module function to fill out data structure of type VimanThetaMat
    """

    obj.thetaMatOrigin.nx = t3d.matrix[0][0]
    obj.thetaMatOrigin.ox = t3d.matrix[1][0]
    obj.thetaMatOrigin.ax = t3d.matrix[2][0]
    obj.thetaMatOrigin.px = t3d.x

    obj.thetaMatOrigin.ny = t3d.matrix[0][1]
    obj.thetaMatOrigin.oy = t3d.matrix[1][1]
    obj.thetaMatOrigin.ay = t3d.matrix[2][1]
    obj.thetaMatOrigin.py = t3d.y

    obj.thetaMatOrigin.nz = t3d.matrix[0][2]
    obj.thetaMatOrigin.oz = t3d.matrix[1][2]
    obj.thetaMatOrigin.az = t3d.matrix[2][2]
    obj.thetaMatOrigin.pz = t3d.z

def _fill_robot_matrix(obj, robot, obj_3dpose):
    """ Fill the world matix part of the structure

    This function receives the Blender rotation matrix and position of an object
    It calls a module function to fill out data structure of type VimanThetaMat
    """
    robot3d = robot.position_3d

    t3d = robot3d.transformation3d_with(obj_3dpose)

    obj.thetaMatRobot.nx = t3d.matrix[0][0]
    obj.thetaMatRobot.ox = t3d.matrix[1][0]
    obj.thetaMatRobot.ax = t3d.matrix[2][0]
    obj.thetaMatRobot.px = t3d.x

    obj.thetaMatRobot.ny = t3d.matrix[0][1]
    obj.thetaMatRobot.oy = t3d.matrix[1][1]
    obj.thetaMatRobot.ay = t3d.matrix[2][1]
    obj.thetaMatRobot.py = t3d.y

    obj.thetaMatRobot.nz = t3d.matrix[0][2]
    obj.thetaMatRobot.oz = t3d.matrix[1][2]
    obj.thetaMatRobot.az = t3d.matrix[2][2]
    obj.thetaMatRobot.pz = t3d.z


class VimanPoster(PocolibsDataStreamOutput):
    _type_name = "VimanObjectPublicArray"
    _type_url = "http://trac.laas.fr/git/viman-genom/tree/vimanStruct.h#n139"

    def initialize(self):
        super(self.__class__, self).initialize(VimanObjectPublicArray)

        # Initialise the object
        self.obj = VimanObjectPublicArray()

        self.scene_object_list = []

        #If provided, read the list of ARToolkit tags to fill the list of objects
        #to export.
        self.scene_object_list += _read_object_list()

        #Complete the list with the objects already tracked by the semantic cam.
        if 'passiveObjectsDict' in blenderapi.persistantstorage():
            self.scene_object_list += [obj['label'] for obj in blenderapi.persistantstorage().passiveObjectsDict.values()]

        if not self.scene_object_list:
            logger.error("No VIMAN object to track. Make sure some objects have " +\
                   "the game property 'Object' defined. Disabling poster for now.")
            return None

        # Init the data structures used by this poster
        self.obj.nbObjects = len(self.scene_object_list)
        i = 0
        for object in self.scene_object_list:
            logger.info("Adding " + object + " to the objects tracked by VIMAN")
            self.obj.objects[i] = bytes(str(object), 'utf-8')
            i = i + 1

    def _read_object_list(self):
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

    def default(self, ci):
        seen_objects = [obj['name'] for obj in self.data['visible_objects']]
        parent = self.component_instance.robot_parent

        i = 0
        for object_id in self.scene_object_list:
            try:
                t = time.time()
                tacq_sec = int(t)
                tacq_usec = int((t - tacq_sec) * 1000)
                self.obj.objects[i].tacq_sec = tacq_sec
                self.obj.objects[i].tacq_usec = tacq_usec

                if object_id in seen_objects:
                    obj = passive_objects.obj_from_label(object_id)

                    position_3d = Transformation3d(obj)
                    logger.debug("VIMAN " + object_id + "(" + obj.name +
                                 ") is visible at " + str(position_3d))
                    self.obj.objects[i].found_Stereo = 1
                    _fill_world_matrix(self.obj.objects[i], position_3d)
                    _fill_robot_matrix(self.obj.objects[i], parent, position_3d)
                else:
                    self.obj.objects[i].found_Stereo = 0
            except KeyError as detail:
                logger.debug("WARNING: Object %s not found in the scene" % detail)
            i = i + 1
        self.write(self.obj)


