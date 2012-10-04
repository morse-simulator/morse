import os, random
from time import sleep

import hla
import hla.rti as rti
import hla.omt as fom

#Defines the 'MorseVector' type that will be transfered on the HLA federation.
MorseVector = fom.HLAfixedArray("MorseVector", fom.HLAfloat32LE, 3)

class MorseHLAClient():
    def __init__(self, robot_name, host="localhost", port=60400):
        self.fom = "morse.fed"
        self.federation = "MORSE"
        self.robot = None
        # Setup environment
        if os.getenv("CERTI_HTTP_PROXY") == None:
            os.environ["CERTI_HTTP_PROXY"] = ""
        os.environ["CERTI_HOST"] = str(host)
        os.environ["CERTI_TCP_PORT"] = str(port)
        # Create Ambassador
        try:
            self.rtia = rti.RTIAmbassador()
            print("Creating MORSE Federation...")
            try:
                self.rtia.createFederationExecution(self.federation, self.fom)
            except rti.FederationExecutionAlreadyExists:
                print("%s federation already exists", self.federation)
            except rti.CouldNotOpenFED:
                print("FED file not found! " + \
                      "Please check that the '.fed' file is in the CERTI " + \
                      "search path of RTIg.")
                return None
            except rti.ErrorReadingFED:
                print("Error when reading FED file! " + \
                      "Please check the '.fed' file syntax.")
                return None
            print("Joining MORSE Federation...")
            try:
                self.morse_ambassador = rti.FederateAmbassador()
                self.rtia.joinFederationExecution("client-hla", self.federation, self.morse_ambassador)
            except rti.FederateAlreadyExecutionMember:
                print("A Federate with name %s has already registered."+\
                      " Change the name of your federate or " + \
                      "check your federation architecture.", self.node_name)
                return None
            except rti.CouldNotOpenFED:
                print("FED file not found! Please check that the " + \
                      "'.fed' file is in the CERTI search path.")
                return None
            except rti.ErrorReadingFED:
                print("Error when reading FED file! "+ \
                      "Please check the '.fed' file syntax.")
                return None
            # Get class/attribute handlers
            try:
                self.robot_t = self.rtia.getObjectClassHandle("Robot")
                self.position_t = self.rtia.getAttributeHandle("position", self.robot_t)
                self.orientation_t = self.rtia.getAttributeHandle("orientation", self.robot_t)
            except rti.NameNotFound:
                print("'Robot' (or attributes) not declared in FOM." + \
                      "Your '.fed' file may not be up-to-date.")
                return None
            # Declare a robot
            self.rtia.publishObjectClass(self.robot_t, [self.position_t, self.orientation_t])
            self.robot = self.rtia.registerObjectInstance(self.robot_t, robot_name)
            print("Pose of robot %s will be published on the %s federation.", robot_name, self.federation)
            print("MorseAmbassador initialized")
        except Exception as error:
            print("Error when connecting to the RTIg: %s." + \
                  "Please check your HLA network configuration.", error)
            return None
            
    def __del__(self):
        print("Resigning from the HLA federation")
        self.rtia.deleteObjectInstance(self.robot, self.rtia.getObjectInstanceName(self.robot))
        self.rtia.resignFederationExecution(hla.rti.ResignAction.DeleteObjectsAndReleaseAttributes)
            
    def send(self, x, y):
        hla_att = {self.position_t: MorseVector.pack([x, y, 0]),
                   self.orientation_t: MorseVector.pack([0, 0, 0])
                   }
        print("Sending ATRV position ", x, ", ", y)
        self.rtia.updateAttributeValues(self.robot, hla_att, "update")
        self.rtia.tick()

# main
if __name__ == "__main__":
    m = MorseHLAClient("ATRV")
    if m != None:
        for k in range(10):
            x = random.randint(0, 7)
            y = random.randint(-5, 0)
            m.send(x, y)
            sleep(1)
