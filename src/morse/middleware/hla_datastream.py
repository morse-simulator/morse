import logging; logger = logging.getLogger("morse." + __name__)
import hla.rti as rti

from morse.core.datastream import DatastreamManager
from morse.core import blenderapi

class MorseBaseAmbassador(rti.FederateAmbassador):
    def __init__(self, rtia, federation, time_sync):
        self._rtia = rtia
        self.federation = federation
        self._time_sync = time_sync

        self.synchronisation_points = {}
        self.registred_objects = []

        self._object_handles = {} # string -> obj_handle
        self._attributes_handles = {} # (obj_handle, string) -> attr_handle
        self._attributes_subscribed = {} # obj_handle -> [attr_handle]
        
        self._attributes_values = {} # obj_handle -> { attr_handle -> value }

    def initialize_time_regulation(self):
        self.logical_time = self._rtia.queryFederateTime()
        logger.debug("federation %s time %f" % (self.federation, self.logical_time))

        self.constraint_enabled = False
        self.regulator_enabled = False
        self.granted = False
        self.lookahead = 1.0

        self._rtia.enableTimeConstrained()
        self._rtia.enableTimeRegulation(self.logical_time, self.lookahead)
        while not (self.constraint_enabled and self.regulator_enabled):
            self._rtia.tick(0, self.lookahead)

    def advance_time(self):
        if self._time_sync:
            self.granted = False
            self._rtia.timeAdvanceRequest(self.logical_time + self.lookahead)
            while not self.granted:
                self._rtia.tick(0, self.lookahead)
        else:
            self._rtia.tick()

    def wait_until_sync(self, label):
        # Make sure that we receive the announce sync point
        while not label in self.synchronisation_points:
            self._rtia.tick()

        self._rtia.synchronizationPointAchieved(label)
        while not self.synchronisation_points[label]:
            self._rtia.tick()

    def register_object(self, handle, name):
        obj = self._rtia.registerObjectInstance(handle, name)
        self.registred_objects.append(obj)
        return obj

    def terminate(self):
        for obj in self.registred_objects:
            self._rtia.deleteObjectInstance(obj,
                self._rtia.getObjectInstanceName(obj))

    def object_handle(self, name):
        handle = self._object_handles.get(name, None)
        if not handle:
            handle = self._rtia.getObjectClassHandle(name)
            self._object_handles[name] = handle
        return handle

    def attribute_handle(self, name, obj_handle):
        handle = self._attributes_handles.get((obj_handle, name), None)
        if not handle:
            handle = self._rtia.getAttributeHandle(name, obj_handle)
            self._attributes_handles[(obj_handle, name)] = handle
        return handle

    def suscribe_attributes(self, obj_handle, attr_handles):
        logger.debug("suscribe_attributes %s => %s" % (obj_handle, attr_handles))
        curr_tracked_attr = set(self._attributes_subscribed.get(obj_handle, []))
        res = list(curr_tracked_attr.union(attr_handles))
        self._attributes_subscribed[obj_handle] = res

        self._rtia.subscribeObjectClassAttributes(obj_handle, res)

    def get_attributes(self, obj_name):
        for key, attr in self._attributes_values.items():
            if self._rtia.getObjectInstanceName(key) == obj_name:
                return attr

        return None

    def update_attribute(self, obj_handle, value):
        if self._time_sync:
            self._rtia.updateAttributeValues(obj_handle, value, "morse_update",
                                             self.logical_time + self.lookahead)
        else:
            self._rtia.updateAttributeValues(obj_handle, value, "morse_update")

    # Callbacks for FedereteAmbassadors 
    def discoverObjectInstance(self, obj, objectclass, name):
        logger.debug("DISCOVER %s %s %s" % (name, obj, objectclass))
        subscribed_attributes = self._attributes_subscribed.get(objectclass, None)
        if subscribed_attributes:
            self._rtia.requestObjectAttributeValueUpdate(obj, subscribed_attributes)
            default_value = {}
            for attr in subscribed_attributes:
                default_value[attr] =  None
            self._attributes_values[obj] = default_value

    def reflectAttributeValues(self, obj, attributes, tag, order, transport, time=None, retraction=None):
        logger.debug("reflectAttributeValues for %s %s" % (self._rtia.getObjectInstanceName(obj), attributes))
        attr_entry = self._attributes_values.get(obj, None)
        if not attr_entry:
            return
        for key in attr_entry.keys():
            if key in attributes:
                attr_entry[key] = attributes[key]

    def timeConstrainedEnabled(self, time):
        logger.debug("Constrained at time %f" % time)
        self.logical_time = time
        self.constraint_enabled  = True
    
    def timeRegulationEnabled(self, time):
        logger.debug("Regulator at time %f" % time)
        self.logical_time = time
        self.regulator_enabled = True
    
    def timeAdvanceGrant(self, time):
        logger.debug("time Advance granted %f" % time)
        self.logical_time = time
        self.granted = True

    def announceSynchronizationPoint(self, label, tag):
        self.synchronisation_points[label] = False

    def federationSynchronized(self, label):
        self.synchronisation_points[label] = True


class HLABaseNode:
    def __init__(self, klass, fom, node_name, federation, sync_point, time_sync):
        """
        Initializes HLA (connection to RTIg, FOM file, publish robots...)
        """

        logger.info("Initializing HLA node.")

        try:
            logger.debug("Creating RTIA...")
            self.rtia = rti.RTIAmbassador()
            logger.debug("RTIA created!")
            try:
                self.rtia.createFederationExecution(federation, fom)
                logger.info("%s federation created", federation)
            except rti.FederationExecutionAlreadyExists:
                logger.debug("%s federation already exists", federation)
            except rti.CouldNotOpenFED:
                logger.error("FED file not found! " + \
                    "Please check that the '.fed' file is in the CERTI " + \
                    "search path of RTIg.")
                raise
            except rti.ErrorReadingFED:
                logger.error("Error when reading FED file! " + \
                    "Please check the '.fed' file syntax.")
                raise
            logger.debug("Creating MorseAmbassador...")
            self.morse_ambassador = klass(self.rtia, federation, time_sync)
            try:
                self.rtia.joinFederationExecution(node_name, 
                    federation, self.morse_ambassador)
            except rti.FederateAlreadyExecutionMember:
                logger.error("A Federate with name %s has already registered."+\
                    " Change the name of your federate or " + \
                    "check your federation architecture.", self.node_name)
                raise
            except rti.CouldNotOpenFED:
                logger.error("FED file not found! Please check that the " + \
                    "'.fed' file is in the CERTI search path.")
                raise
            except rti.ErrorReadingFED:
                logger.error("Error when reading FED file! "+ \
                    "Please check the '.fed' file syntax.")
                raise
            logger.info("HLA middleware initialized.")

        except Exception as error:
            logger.error("Error when connecting to the RTIg: %s." + \
                "Please check your HLA network configuration.", error)
            raise

        if sync_point:
            self.morse_ambassador.wait_until_sync(sync_point)

        if time_sync:
            self.morse_ambassador.initialize_time_regulation()
            
    def __del__(self):
        """
        Close all open HLA connections.
        """
        logger.info("Resigning from the HLA federation")
        if self.morse_ambassador:
            del self.morse_ambassador
        self.rtia.resignFederationExecution(
            rti.ResignAction.DeleteObjectsAndReleaseAttributes)

class HLADatastreamManager(DatastreamManager):
    """ External communication using sockets. """

    def __init__(self, args, kwargs):
        """ Initialize the socket connections """
        # Call the constructor of the parent class
        DatastreamManager.__init__(self, args, kwargs)

        try:
            fom = kwargs["fom"]
            node_name = kwargs["name"]
            federation = kwargs["federation"]
            sync_point = kwargs.get("sync_point", None)
            time_sync = kwargs.get("time_sync", False)

            self.node = HLABaseNode(MorseBaseAmbassador, fom, node_name,
                                    federation, sync_point, time_sync)
        except KeyError as error:
            logger.error("One of [fom, name, federation] attribute is not configured: "
                         "Cannot create HLADatastreamManager")
            raise

    def register_component(self, component_name, component_instance, mw_data):
        """ Open the port used to communicate by the specified component.
        """

        mw_data[2]['__hla_node'] = self.node

        DatastreamManager.register_component(self, component_name,
                                                   component_instance, mw_data)

    def action(self):
        self.node.morse_ambassador.advance_time()
