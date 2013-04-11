import logging; logger = logging.getLogger("morse." + __name__)
import mathutils
import os

from morse.core import blenderapi
from morse.core.exceptions import MorseMultinodeError
from morse.core.multinode import SimulationNodeClass

logger.setLevel(logging.INFO)

try:
    import hla
    import hla.rti as rti
    import hla.omt as fom
except (ImportError, SyntaxError):
    logger.error("No HLA binding found or imported!")
    raise MorseMultinodeError("'hla' python not found, or has syntax errors")

"""
Defines the 'MorseVector' type that will be transfered on the HLA federation.
"""
MorseVector = fom.HLAfixedArray("MorseVector", fom.HLAfloat32LE, 3)

class MorseAmbassador(rti.FederateAmbassador):
    """
    The Federate Ambassador of the MORSE node.
    
    """
    def __init__(self, rtia, federation, time_regulation, time):
        self.objects = []
        self.rtia_ = rtia
        self.constrained = False
        self.regulator = False
        self.tag = time_regulation
        self.federation = federation
        self.current_time = time
        self.lookahead = 0
        logger.debug("MorseAmbassador created.")
    
    def initialize(self):
        try:
            self.out_robot = self.rtia_.getObjectClassHandle("Robot")
            self.out_position = self.rtia_.getAttributeHandle("position",
                self.out_robot)
            self.out_orientation = self.rtia_.getAttributeHandle(
                "orientation", self.out_robot)
        except rti.NameNotFound:
            logger.error("'Robot' (or attributes) not declared in FOM." + \
                "Your '.fed' file may not be up-to-date.")
            return False
        
        self.rtia_.publishObjectClass(self.out_robot, 
            [self.out_position, self.out_orientation])

        robot_dict = blenderapi.persistantstorage().robotDict
        for obj, local_robot_data in robot_dict.items():
            self.objects.append(self.rtia_.registerObjectInstance(
                    self.out_robot, obj.name))
            logger.info(
                "Pose of robot %s will be published on the %s federation.", 
                obj.name, self.federation)
        
        self.in_robot = self.rtia_.getObjectClassHandle("Robot")
        self.in_position = self.rtia_.getAttributeHandle("position", 
            self.in_robot)
        self.in_orientation = self.rtia_.getAttributeHandle("orientation", 
            self.in_robot)
        self.rtia_.subscribeObjectClassAttributes(self.in_robot, 
            [self.in_position, self.in_orientation])
        # TSO initialization
        if self.tag:
            self.tag = False
            self.lookahead = 1
            self.current_time = self.rtia_.queryFederateTime()
            logger.debug("Initial Federate time is %s", self.current_time)
            self.rtia_.enableTimeConstrained()
            self.rtia_.enableTimeRegulation(self.current_time, self.lookahead)
            while (not self.constrained and not self.regulator and not self.tag):
                self.rtia_.tick(0, 1)
        logger.debug("MorseAmbassador initialized")

    def terminate(self):
        for obj in self.objects:
            self.rtia_.deleteObjectInstance(obj, 
                self.rtia_.getObjectInstanceName(obj))

    def discoverObjectInstance(self, object, objectclass, name):
        logger.info(
            "Robot %s will have its pose reflected in the current node...", 
            name)

    def reflectAttributeValues(self, object, attributes, tag, order, transport, 
                               time=None, retraction=None):
        scene = blenderapi.scene()
        obj_name = self.rtia_.getObjectInstanceName(object)
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
    
    def timeConstrainedEnabled(self, time):
        logger.debug("Constrained at time %s", time)
        self.current_time = time
        self.constrained = True
    
    def timeRegulationEnabled(self, time):
        logger.debug("Regulator at time %s", time)
        self.current_time = time
        self.regulator = True
    
    def timeAdvanceGrant(self, time):
        self.current_time = time
        self.tag = True
    
class HLANode(SimulationNodeClass):
    """
    Implements multinode simulation using HLA.
    """
    
    time_sync = False
    fom = "morse.fed"
    federation = "MORSE"
    
    def initialize(self):
        """
        Initializes HLA (connection to RTIg, FOM file, publish robots...)
        
        """
        logger.info("Initializing HLA node.")
        if os.getenv("CERTI_HTTP_PROXY") == None:
            os.environ["CERTI_HTTP_PROXY"] = ""
        os.environ["CERTI_HOST"] = str(self.host)
        os.environ["CERTI_TCP_PORT"] = str(self.port)
        logger.debug("CERTI_HTTP_PROXY= %s", os.environ["CERTI_HTTP_PROXY"])
        logger.debug("CERTI_HOST= %s", os.environ["CERTI_HOST"])
        logger.debug("CERTI_TCP_PORT= %s", os.environ["CERTI_TCP_PORT"])
        try:
            logger.debug("Creating RTIA...")
            self.rtia = rti.RTIAmbassador()
            logger.debug("RTIA created!")
            try:
                self.rtia.createFederationExecution(self.federation, self.fom)
                logger.info("%s federation created", self.federation)
            except rti.FederationExecutionAlreadyExists:
                logger.debug("%s federation already exists", self.federation)
            except rti.CouldNotOpenFED:
                logger.error("FED file not found! " + \
                    "Please check that the '.fed' file is in the CERTI " + \
                    "search path of RTIg.")
                return False
            except rti.ErrorReadingFED:
                logger.error("Error when reading FED file! " + \
                    "Please check the '.fed' file syntax.")
                return False
            logger.debug("Creating MorseAmbassador...")
            self.morse_ambassador = MorseAmbassador(self.rtia, self.federation,
                self.time_sync, 0)
            try:
                self.rtia.joinFederationExecution(self.node_name, 
                    self.federation, self.morse_ambassador)
            except rti.FederateAlreadyExecutionMember:
                logger.error("A Federate with name %s has already registered."+\
                    " Change the name of your federate or " + \
                    "check your federation architecture.", self.node_name)
                return False
            except rti.CouldNotOpenFED:
                logger.error("FED file not found! Please check that the " + \
                    "'.fed' file is in the CERTI search path.")
                return False
            except rti.ErrorReadingFED:
                logger.error("Error when reading FED file! "+ \
                    "Please check the '.fed' file syntax.")
                return False
            if self.morse_ambassador.initialize() == False:
                return False
            logger.info("HLA middleware initialized.")
        except Exception as error:
            logger.error("Error when connecting to the RTIg: %s." + \
                "Please check your HLA network configuration.", error)
            raise
            
    def finalize(self):
        """
        Close all open HLA connections.
        
        """
        logger.info("Resigning from the HLA federation")
        if self.morse_ambassador:
            self.morse_ambassador.terminate()
        self.rtia.resignFederationExecution(
            rti.ResignAction.DeleteObjectsAndReleaseAttributes)
            
    def synchronize(self):
        self.morse_ambassador.tag = False
        scene = blenderapi.scene()
        t = self.morse_ambassador.current_time + self.morse_ambassador.lookahead
        for obj in self.morse_ambassador.objects:
            obj_name = self.rtia.getObjectInstanceName(obj)
            obj_pos = scene.objects[obj_name].worldPosition.to_tuple()
            obj_ori = scene.objects[obj_name].worldOrientation.to_euler()
            hla_att = {
                self.morse_ambassador.out_position:
                    MorseVector.pack([obj_pos[0], obj_pos[1], obj_pos[2]]),
                self.morse_ambassador.out_orientation:
                    MorseVector.pack([obj_ori.x, obj_ori.y, obj_ori.z])}
            try:
                self.rtia.updateAttributeValues(obj, hla_att, "update", t)
            except rti.InvalidFederationTime:
                logger.debug("Invalid time for UAV: %s; Federation time is %s",
                    t, self.rtia.queryFederateTime())
        if self.time_sync:
            self.rtia.timeAdvanceRequest(t)
            while (not self.morse_ambassador.tag):
                self.rtia.tick(0, 1)
            logger.debug("Node simulation time:" + \
                self.morse_ambassador.current_time)
        else:
            self.rtia.tick()
