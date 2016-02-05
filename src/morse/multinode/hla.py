import logging; logger = logging.getLogger("morse." + __name__)
import os

from morse.core import blenderapi, mathutils
from morse.core.exceptions import MorseMultinodeError
from morse.core.multinode import SimulationNodeClass
from morse.middleware.hla_datastream import MorseBaseAmbassador, HLABaseNode

try:
    import hla.rti as rti
    import hla.omt as fom
except (ImportError, SyntaxError):
    logger.error("No HLA binding found or imported!")
    raise MorseMultinodeError("'hla' python not found, or has syntax errors")

"""
Defines the 'MorseVector' type that will be transfered on the HLA federation.
"""
MorseVector = fom.HLAfixedArray("MorseVector", fom.HLAfloat32LE, 3)

class MorseAmbassador(MorseBaseAmbassador):
    """
    The Federate Ambassador of the MORSE node.
    
    """
    def __init__(self, rtia, federation, time_regulation, timestep, lookahead):
        MorseBaseAmbassador.__init__(self, rtia, federation, time_regulation,
                                           timestep, lookahead)
        logger.debug("MorseAmbassador created.")
    
    def initialize(self):
        try:
            out_robot = self.object_handle("Robot")
            self.out_position = self.attribute_handle("position", out_robot)
            self.out_orientation = self.attribute_handle("orientation", out_robot)
        except rti.NameNotFound:
            logger.error("'Robot' (or attributes) not declared in FOM." + \
                "Your '.fed' file may not be up-to-date.")
            return False

        self._rtia.publishObjectClass(out_robot,
            [self.out_position, self.out_orientation])

        robot_dict = blenderapi.persistantstorage().robotDict
        for obj in robot_dict.keys():
            self.register_object(out_robot, obj.name)
            logger.info(
                "Pose of robot %s will be published on the %s federation.", 
                obj.name, self.federation)
        
        in_robot = self.object_handle("Robot")
        self.in_position = self.attribute_handle("position", in_robot)
        self.in_orientation = self.attribute_handle("orientation", in_robot)
        self.suscribe_attributes(obj.name, in_robot, [self.in_position, self.in_orientation])

    def discoverObjectInstance(self, object, objectclass, name):
        logger.info(
            "Robot %s will have its pose reflected in the current node...", 
            name)

    def reflectAttributeValues(self, object, attributes, tag, order, transport, 
                               time=None, retraction=None):
        scene = blenderapi.scene()
        obj_name = self._rtia.getObjectInstanceName(object)
        logger.debug("RAV %s", obj_name)
        try:
            obj = scene.objects[obj_name]
            if self.in_position in attributes:
                pos, offset = MorseVector.unpack(attributes[self.in_position])
                # Update the positions of the robots
                obj.worldPosition = pos
            if self.in_orientation in attributes:
                ori, offset = MorseVector.unpack(attributes[self.in_orientation])
                # Update the orientations of the robots
                obj.worldOrientation = mathutils.Euler(ori).to_matrix()
        except KeyError as detail:
            logger.debug("Robot %s not found in this simulation scenario," + \
                "but present in another node. Ignoring it!", obj_name)
    
class HLANode(SimulationNodeClass):
    """
    Implements multinode simulation using HLA.
    """
    
    time_sync = False
    fom = "morse.fed"
    federation = "MORSE"
    
    def initialize(self):
        logger.info("Initializing HLA node.")
        if os.getenv("CERTI_HTTP_PROXY") is None:
            os.environ["CERTI_HTTP_PROXY"] = ""
        os.environ["CERTI_HOST"] = str(self.host)
        os.environ["CERTI_TCP_PORT"] = str(self.port)

        self.node = HLABaseNode(MorseAmbassador, self.fom, self.node_name,
                self.federation, None, None, self.time_sync, 1.0, 1.0)

        self.node.morse_ambassador.initialize()

    def finalize(self):
        """
        Close all open HLA connections.
        """
        del self.node

    def synchronize(self):
        if not self.node:
            return

        scene = blenderapi.scene()
        for obj in self.node.morse_ambassador.registred_objects.values():
            obj_name = self.node.rtia.getObjectInstanceName(obj)
            obj_pos = scene.objects[obj_name].worldPosition.to_tuple()
            obj_ori = scene.objects[obj_name].worldOrientation.to_euler()
            hla_att = {
                self.node.morse_ambassador.out_position:
                    MorseVector.pack([obj_pos[0], obj_pos[1], obj_pos[2]]),
                self.node.morse_ambassador.out_orientation:
                    MorseVector.pack([obj_ori.x, obj_ori.y, obj_ori.z])}
            self.node.morse_ambassador.update_attribute(obj, hla_att)
        self.node.morse_ambassador.advance_time()
