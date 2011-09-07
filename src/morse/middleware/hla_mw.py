import mathutils
import struct
import os
import logging; logger = logging.getLogger("morse.HLA")
logging.basicConfig(level=logging.DEBUG)

import GameLogic

import morse.core.middleware

try:
    import hla.rti as rti
    import hla.omt as fom
except (ImportError, SyntaxError):
    logger.warning("No HLA binding found or imported: the HLA middleware will be disabled.")
    rti = None

### HLA Environment Configuration:
certi_env = {}
certi_env["Federation"] = "Morse" # Federation name
certi_env["Federate"] = (os.uname()[1] + str(os.getpid())) # Federate name
certi_env["FOM"] = "morse.fed" # FOM file
certi_env["CERTI_HOST"] = "localhost" # HOST
if os.getenv("CERTI_HOST") != None:
    certi_env["CERTI_HOST"] = os.environ["CERTI_HOST"]
certi_env["TimeRegulation"] = False # TimeConstrained

def configureHLAEnvironment():
    """
    Configure the HLA network environment.
    Uses the Game Properties of the HLA_Empty object if defined,
    default values otherwise.
    
    """
    logger.info("Initializing configuration")
    if os.getenv("CERTI_HTTP_PROXY") == None:
        os.environ["CERTI_HTTP_PROXY"] = ""
    try:
        scene = GameLogic.getCurrentScene()
        certi_object = scene.objects["HLA_Empty"]
        for k in certi_env.keys():
            try:
                v = certi_object[k]
                certi_env[k] = v
                logger.debug("%s: %s", k, v)
            except KeyError:
                logger.debug("No property for %s; using %s", k, certi_env[k])
    except KeyError:
        log.error("The HLA_Empty object has not been found on current scene!")
    os.environ["CERTI_HOST"] = certi_env["CERTI_HOST"]

if rti:
    rtia = 0
    morse_ambassador = 0
    
    class MorseAmbassador(rti.FederateAmbassador):
        """
        The Federate Ambassador of the Morse node.
        
        """
        def __init__(self):
            self.objects = []
            self.constrained = False
            self.regulator = False
            self.TAG = False
            try:
                self.currentTime = GameLogic.getCurrentScene().objects["HLA_Empty"]["Time"]
            except KeyError:
                self.currenTime = 0
            self.lookahead = 0
    
        def initialize(self):
            try:
                self.outRobotHandle = rtia.getObjectClassHandle("Robot")
                self.outPositionHandle = rtia.getAttributeHandle("position", self.outRobotHandle)
                self.outOrientationHandle = rtia.getAttributeHandle("orientation", self.outRobotHandle)
            except rti.NameNotFound:
                logger.error("'Robot' (or attributes) not declared in FOM. Your '.fed' file may not be up-to-date.")
                return False
            rtia.publishObjectClass(self.outRobotHandle, [self.outPositionHandle, self.outOrientationHandle])
            for obj, local_robot_data in GameLogic.robotDict.items():
                self.objects.append(rtia.registerObjectInstance(self.outRobotHandle, obj.name))
                logger.info("Pose of robot %s will be published on the %s federation.", obj.name, certi_env["Federation"])
            self.inRobotHandle = rtia.getObjectClassHandle("Robot")
            self.inPositionHandle = rtia.getAttributeHandle("position", self.inRobotHandle)
            self.inOrientationHandle = rtia.getAttributeHandle("orientation", self.inRobotHandle)
            rtia.subscribeObjectClassAttributes(self.inRobotHandle, [self.inPositionHandle, self.inOrientationHandle])
            # TSO initialization
            if certi_env["TimeRegulation"]:
                self.lookahead = 1
                self.currentTime = rtia.queryFederateTime()
                logger.debug("Initial Federate time is %s", self.currentTime)
                rtia.enableTimeConstrained()
                rtia.enableTimeRegulation(self.currentTime, self.lookahead)
                while (not self.constrained and not self.regulator and not self.TAG):
                    rtia.tick(0, 1)

        def terminate(self):
            for obj in self.objects:
                rtia.deleteObjectInstance(obj, rtia.getObjectInstanceName(obj))

        def discoverObjectInstance(self, object, objectclass, name):
            logger.debug("DISCOVER %s", name)
            logger.info("Robot %s will have its pose reflected in the current node...", name)

        def reflectAttributeValues(self, object, attributes, tag, order, transport, time=None, retraction=None):
            scene = GameLogic.getCurrentScene()
            obj_name = rtia.getObjectInstanceName(object)
            obj = scene.objects[obj_name]
            try:
                if self.inPositionHandle in attributes:
                    position = struct.unpack('ddd', attributes[self.inPositionHandle])
                    # Update the positions of the robots
                    obj.worldPosition = position
                if self.inOrientationHandle in attributes:
                    orientation = struct.unpack('ddd', attributes[self.inOrientationHandle])
                    # Update the orientations of the robots
                    obj.worldOrientation = mathutils.Euler(orientation).to_matrix()
            except KeyError as detail:
                logger.debug("Robot %s not found in this simulation scenario, but present in another node. Ignoring it!", obj_name)
        
        def timeConstrainedEnabled(self, time):
            logger.debug("Constrained at time %s", time)
            self.currentTime = time
            self.constrained = True
            
        def timeRegulationEnabled(self, time):
            logger.debug("Regulator at time %s", time)
            self.currentTime = time
            self.regulator = True
            
        def timeAdvanceGrant(self, time):
            self.currentTime = time
            self.TAG = True

    def initializeHLA():
        """
        Initializes HLA (connection to RTIg, FOM file, publish robots...)
        
        """
        logger.info("Connecting to the Morse federation")
        global rtia
        global morse_ambassador
        configureHLAEnvironment()
        try:
            rtia = rti.RTIAmbassador()
            try:
                rtia.createFederationExecution(certi_env["Federation"], certi_env["FOM"])
                logger.info("%s federation created", certi_env["Federation"])
            except rti.FederationExecutionAlreadyExists:
                logger.debug("%s federation already exists", certi_env["Federation"])
            except rti.CouldNotOpenFED:
                logger.error("FED file not found! Please check that the '.fed' file is in the CERTI search path of RTIg.")
                return False
            except rti.ErrorReadingFED:
                logger.error("Error when reading FED file! Please check the '.fed' file syntax.")
                return False
            morse_ambassador = MorseAmbassador()
            try:
                rtia.joinFederationExecution(certi_env["Federate"], certi_env["Federation"], morse_ambassador)
            except rti.FederateAlreadyExecutionMember:
                logger.error("A Federate with name %s has already registered. Change the name of your federate or check your federation architecture.",
                             certi_env["Federate"])
                return False
            except rti.CouldNotOpenFED:
                logger.error("FED file not found! Please check that the '.fed' file is in the CERTI search path.")
                return False
            except rti.ErrorReadingFED:
                logger.error("Error when reading FED file! Please check the '.fed' file syntax.")
                return False
            if morse_ambassador.initialize() == False:
                return False
            logger.debug("HLA middleware initialized")
        except BaseException as error:
            logger.error("Error when connecting to the RTIg: %s. Please check your HLA network configuration.", error)
            return False

    def finalizeHLA():
        logger.info("Resigning from the HLA federation")
        morse_ambassador.terminate()
        rtia.resignFederationExecution(rti.ResignAction.DeleteObjectsAndReleaseAttributes)
    
class HLAClass(morse.core.middleware.MorseMiddlewareClass):
    """
    HLA middleware for Morse.
    This HLA class is also used to synchronize several Morse nodes.
    
    """
      
    def __init__(self, obj, parent=None):
        """
        Initialize HLA.
        
        """
        super(self.__class__, self).__init__(obj, parent)
        if rti:
            initializeHLA()
        
    def __del__(self):
        """
        Close all open HLA connections.
        
        """
        self.finalize()
    
    def finalize(self):
        """
        Close all open HLA connections.
        
        """
        if rti:
            finalizeHLA()
        
    def register_component(self, component_name, component_instance, mw_data):
        """
        HLA as a middleware for components has not been implemented yet...
        
        """
        pass

def update_robots():
    """
    Update robots' poses in the HLA federation for multinode synchronization.
    
    """
    if rti:
        morse_ambassador.TAG = False
        scene = GameLogic.getCurrentScene()
        t = morse_ambassador.currentTime + morse_ambassador.lookahead
        for obj in morse_ambassador.objects:
            obj_name = rtia.getObjectInstanceName(obj)
            pos = scene.objects[obj_name].worldPosition.to_tuple()
            ori = scene.objects[obj_name].worldOrientation.to_euler()
            pose = {morse_ambassador.outPositionHandle:struct.pack('ddd', pos[0], pos[1], pos[2]), morse_ambassador.outOrientationHandle:struct.pack('ddd', ori.x, ori.y, ori.z)}
            try:
                rtia.updateAttributeValues(obj, pose, "update", t)
            except rti.InvalidFederationTime:
                logger.debug("Invalid time for UAV: %s; Federation time is %s", t, rtia.queryFederateTime())
        if certi_env["TimeRegulation"]:
            rtia.timeAdvanceRequest(t)
            while (not morse_ambassador.TAG):
                rtia.tick(0, 1)
            scene.objects["HLA_Empty"]["Time"] = morse_ambassador.currentTime
        else:
            rtia.tick()
        