import mathutils
import os
import logging; logger = logging.getLogger("morse." + __name__)
#logger.setLevel(logging.DEBUG)

import GameLogic

import morse.core.middleware as middleware
import morse.blender.main as morse

try:
    import hla.rti as rti
    import hla.omt as fom
except (ImportError, SyntaxError):
    logger.error("No HLA binding found or imported!")
    raise

"""
Defines the 'MorseVector' type that will be transfered on the HLA federation.
"""
MorseVector = fom.HLAfixedArray("MorseVector", fom.HLAfloat32LE, 3)

class MorseAmbassador(rti.FederateAmbassador):
    """
    The Federate Ambassador of the MORSE node.
    
    """
    def __init__(self, rtia, federation_name="MORSE", time_regulation=False):
        self.objects = []
        self.rtia_ = rtia
        self.constrained = False
        self.regulator = False
        self.tag = time_regulation
        self.federation = federation_name
        try:
            scene = GameLogic.getCurrentScene()
            self.current_time = scene.objects["HLA_Empty"]["Time"]
        except KeyError:
            self.current_time = 0
        self.lookahead = 0
        logger.debug("MorseAmbassador created")
    
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
        for obj, local_robot_data in GameLogic.robotDict.items():
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
        logger.debug("DISCOVER %s", name)
        logger.info(
            "Robot %s will have its pose reflected in the current node...", 
            name)

    def reflectAttributeValues(self, object, attributes, tag, order, transport, 
                               time=None, retraction=None):
        scene = GameLogic.getCurrentScene()
        obj_name = self.rtia_.getObjectInstanceName(object)
        obj = scene.objects[obj_name]
        try:
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
    
class HLAClass(middleware.MorseMiddlewareClass):
    """
    HLA middleware for MORSE.
    This HLA class is also used to synchronize several MORSE nodes.
    
    """
    
    def __init__(self, obj, parent=None):
        """
        Initializes HLA (connection to RTIg, FOM file, publish robots...)
        
        """
        super(self.__class__, self).__init__(obj, parent)
        ### HLA Environment Configuration:
        self.certi_env = {}
        self.certi_env["Federation"] = "MORSE" # Federation name
        self.certi_env["Federate"] = (os.uname()[1] + str(os.getpid())) # Federate name
        self.certi_env["FOM"] = "morse.fed" # FOM file
        self.certi_env["CERTI_HOST"] = "localhost" # HOST
        if os.getenv("CERTI_HOST") != None:
            self.certi_env["CERTI_HOST"] = os.environ["CERTI_HOST"]
        self.certi_env["TimeRegulation"] = False # TimeConstrained
        logger.info("Connecting to the MORSE federation")
        self.configureHLA()
        try:
            logger.debug("Creating RTIA...")
            self.rtia = rti.RTIAmbassador()
            logger.debug("RTIA created!")
            try:
                self.rtia.createFederationExecution(self.certi_env["Federation"], 
                    self.certi_env["FOM"])
                logger.info("%s federation created", self.certi_env["Federation"])
            except rti.FederationExecutionAlreadyExists:
                logger.debug("%s federation already exists", 
                    self.certi_env["Federation"])
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
            self.morse_ambassador = MorseAmbassador(self.rtia)
            try:
                self.rtia.joinFederationExecution(self.certi_env["Federate"], 
                    self.certi_env["Federation"], self.morse_ambassador)
            except rti.FederateAlreadyExecutionMember:
                logger.error("A Federate with name %s has already registered."+\
                    " Change the name of your federate or " + \
                    "check your federation architecture.", self.certi_env["Federate"])
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
            logger.debug("HLA middleware initialized")
        except Exception as error:
            logger.error("Error when connecting to the RTIg: %s." + \
                "Please check your HLA network configuration.", error)
            raise
        
    def __del__(self):
        """
        Close all open HLA connections.
        
        """
        self.finalize()
    
    def finalize(self):
        """
        Close all open HLA connections.
        
        """
        logger.info("Resigning from the HLA federation")
        if self.morse_ambassador:
            self.morse_ambassador.terminate()
        self.rtia.resignFederationExecution(
            rti.ResignAction.DeleteObjectsAndReleaseAttributes)
        
    def register_component(self, component_name, component_instance, mw_data):
        """
        HLA as a middleware for components has not been implemented yet...
        
        """
        pass
   
    def configureHLA(self):
        """
        Configure the HLA network environment.
        Uses the Game Properties of the HLA_Empty object if defined,
        default values otherwise.
        
        """
        logger.info("Initializing configuration")
        if os.getenv("CERTI_HTTP_PROXY") == None:
            os.environ["CERTI_HTTP_PROXY"] = ""
        logger.debug("CERTI_HTTP_PROXY= %s", os.environ["CERTI_HTTP_PROXY"])
        try:
            hla = GameLogic.getCurrentScene().objects["HLA_Empty"]
            for k in self.certi_env.keys():
                try:
                    v = hla[k]
                    self.certi_env[k] = v
                    logger.debug("%s= %s", k, v)
                except KeyError:
                    logger.debug("No property for %s; using %s", k, 
                        self.certi_env[k])
        except KeyError:
            log.error("The HLA_Empty object has not been found on current scene!")
        os.environ["CERTI_HOST"] = self.certi_env["CERTI_HOST"]
        logger.debug("CERTI_HOST= %s", os.environ["CERTI_HOST"])
 
    def update_robots(self):
        """
        Update robots' poses in the HLA federation for multinode synchronization.
        
        """
        self.morse_ambassador.tag = False
        scene = GameLogic.getCurrentScene()
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
        if self.certi_env["TimeRegulation"]:
            self.rtia.timeAdvanceRequest(t)
            while (not self.morse_ambassador.tag):
                self.rtia.tick(0, 1)
            scene.objects["HLA_Empty"]["Time"] = self.morse_ambassador.current_time
        else:
            self.rtia.tick()

def update_robots():
    """
    Call the HLA_Empty.update_robots method.
    """
    #hla = GameLogic.mwDict['HLA_Empty']
    hla = morse.get_middleware_of_type("HLAClass")
    hla.update_robots()
