import logging; logger = logging.getLogger("morse." + __name__)
import hla.rti as rti
import sys

from morse.core.datastream import DatastreamManager
from morse.core import blenderapi

class MorseBaseAmbassador(rti.FederateAmbassador):
    def __init__(self, rtia, federation, time_sync, timestep, lookahead):
        self._rtia = rtia
        self.federation = federation
        self._time_sync = time_sync

        self.synchronisation_points = {}
        self.registred_objects = {} # name -> obj_handle
        self.registred_class_ref = {} # obj_handle -> int

        self._object_handles = {} # string -> obj_handle
        self._attributes_handles = {} # (obj_handle, string) -> attr_handle
        self._objects_discovered = {} # obj_name -> obj
        self._attributes_subscribed = {} # obj_name -> [attr_handle]
        self._attributes_published = {} # obj_name -> [attr_handle]
        
        self._attributes_values = {} # obj_name -> { attr_handle -> value }

        self.timestep = timestep
        self.lookahead = lookahead

    def initialize_time_regulation(self):
        self.logical_time = self._rtia.queryFederateTime()
        logger.debug("federation %s time %f timestep %f" % 
                (self.federation, self.logical_time, self.timestep))

        self.constraint_enabled = False
        self.regulator_enabled = False
        self.granted = False

        self._rtia.enableTimeConstrained()
        self._rtia.enableTimeRegulation(self.logical_time, self.timestep)
        while not (self.constraint_enabled and self.regulator_enabled):
            self._rtia.tick(0, self.lookahead)

    def advance_time(self):
        if self._time_sync:
            self.granted = False
            self._rtia.timeAdvanceRequest(self.logical_time + self.timestep)
            while not self.granted:
                self._rtia.tick(0, self.lookahead)
        else:
            self._rtia.tick()

    def register_sync_point(self, label):
        self._rtia.registerFederationSynchronizationPoint(label,
                "Waiting for other simulators")

    def wait_until_sync(self, label):
        # Make sure that we receive the announce sync point
        while not label in self.synchronisation_points:
            self._rtia.tick()

        self._rtia.synchronizationPointAchieved(label)
        while not self.synchronisation_points[label]:
            self._rtia.tick()

    def register_object(self, handle, name):
        logger.debug("REGISTER object %s => %s" % (name, handle))
        obj = self._rtia.registerObjectInstance(handle, name)
        self.registred_objects[name] = obj
        return obj

    def delete_object(self, name):
        self._rtia.deleteObjectInstance(
                self.registred_objects[name],
                name)
        del self.registred_objects[name]

    def get_object(self, name):
        if name in self.registred_objects:
            return self.registred_objects[name]

        if name in self._objects_discovered:
            return self._objects_discovered[name]

        return None

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

    def suscribe_attributes(self, name, obj_handle, attr_handles):
        logger.debug("suscribe_attributes %s %s => %s" % (name, obj_handle, attr_handles))
        curr_tracked_attr = set(self._attributes_subscribed.get(name, []))
        res = list(curr_tracked_attr.union(attr_handles))
        self._attributes_subscribed[name] = res

        self._rtia.subscribeObjectClassAttributes(obj_handle, res)
        ref_cnt = self.registred_class_ref.get(obj_handle, 0)
        self.registred_class_ref[obj_handle] = ref_cnt + 1
        logger.debug("registred_class_ref %s => %d" % (obj_handle, ref_cnt + 1))

    def publish_attributes(self, name, obj_handle, attr_handles):
        logger.debug("publish_attributes %s %s" % (name, attr_handles))
        curr_tracked_attr = set(self._attributes_published.get(name, []))
        res = list(curr_tracked_attr.union(attr_handles))
        self._attributes_published[name] = res

        self._rtia.publishObjectClass(obj_handle, attr_handles)

    def unsuscribe_attributes(self, obj_handle):
        logger.debug("unsuscribe_attributes %s" % (obj_handle))

        if not obj_handle in self.registred_class_ref:
            return

        self.registred_class_ref[obj_handle] -= 1
        logger.debug("registred_class_ref %s => %d" % (obj_handle,
            self.registred_class_ref[obj_handle]))
        if self.registred_class_ref[obj_handle] == 0:
            self._rtia.unsubscribeObjectClass(obj_handle)
            del self.registred_class_ref[obj_handle]

    def get_attributes(self, obj_name):
        return self._attributes_values.get(obj_name, None)

    def update_attribute(self, obj_handle, value):
        logger.debug("update_attributes %s for %s" % (value, obj_handle))
        if self._time_sync:
            self._rtia.updateAttributeValues(obj_handle, value, "morse_update",
                                             self.logical_time + self.timestep)
        else:
            self._rtia.updateAttributeValues(obj_handle, value, "morse_update")

    # Callbacks for FedereteAmbassadors 
    def discoverObjectInstance(self, obj, objectclass, name):
        logger.debug("DISCOVER %s %s %s" % (name, obj, objectclass))
        self._objects_discovered[name] = obj

        subscribed_attributes = self._attributes_subscribed.get(name, None)
        if subscribed_attributes:
            self._rtia.requestObjectAttributeValueUpdate(obj, subscribed_attributes)
            default_value = {}
            for attr in subscribed_attributes:
                default_value[attr] =  None
            self._attributes_values[name] = default_value

        published_attributes = self._attributes_published.get(name, None)
        if published_attributes:
            logger.debug("attributeOwnershipAcquisition %s %s %s" % (name, obj, published_attributes))
            self._rtia.attributeOwnershipAcquisition(obj, published_attributes, "morse_owner")

    def attributeOwnershipAcquisitionNotification(self, obj, attr):
        obj_name = self._rtia.getObjectInstanceName(obj)
        logger.debug("attributeOwnershipAcquisitionNotification %s %s %s" % (obj, obj_name, attr))

    def reflectAttributeValues(self, obj, attributes, tag, order, transport, time=None, retraction=None):
        try:
            obj_name = self._rtia.getObjectInstanceName(obj)
            logger.debug("reflectAttributeValues for %s %s" % (obj_name, attributes))
            attr_entry = self._attributes_values.get(obj_name, None)
            if not attr_entry:
                return
            for key in attr_entry.keys():
                if key in attributes:
                    attr_entry[key] = attributes[key]
        except rti.ObjectNotKnown:
            logger.warning("Receive an RAV for object %s but it is not anymore "
                           "in the simulation" % obj)

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
    def __init__(self, klass, fom, node_name, federation, sync_point, 
                       sync_register, time_sync, timestep, lookahead):
        """
        Initializes HLA (connection to RTIg, FOM file, publish robots...)
        """

        logger.info("Initializing HLA node.")

        self._federation = federation
        self._sync_point = sync_point
        self._sync_register = sync_register
        self._time_sync = time_sync

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
            self.morse_ambassador = klass(self.rtia, federation, time_sync, timestep, lookahead)
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


    def init_time(self):
        if self._sync_point:
            if self._sync_register:
                self.morse_ambassador.register_sync_point(self._sync_point)
                print("Press ENTER when all simulators are ready")
                sys.stdin.read(1)
            self.morse_ambassador.wait_until_sync(self._sync_point)

        if self._time_sync:
            self.morse_ambassador.initialize_time_regulation()
            
    def finalize(self):
        """
        Close all open HLA connections.
        """
        logger.info("Resigning from the HLA federation")
        if self.morse_ambassador:
            del self.morse_ambassador
        self.rtia.resignFederationExecution(
            rti.ResignAction.DeleteObjectsAndReleaseAttributes)
        try:
            self.rtia.destroyFederationExecution(self._federation)
        except:
            pass
        del self.rtia

class HLADatastreamManager(DatastreamManager):
    """ External communication using sockets. """

    def __init__(self, args, kwargs):
        """ Initialize the socket connections """
        # Call the constructor of the parent class
        DatastreamManager.__init__(self, args, kwargs)
        self._time_initialized = False

        try:
            fom = kwargs["fom"]
            node_name = kwargs["name"]
            federation = kwargs["federation"]
            sync_point = kwargs.get("sync_point", None)
            sync_register = kwargs.get("sync_register", False)
            self.time_sync = kwargs.get("time_sync", False)
            timestep = kwargs.get("timestep", 1.0 / blenderapi.getfrequency())
            lookahead = kwargs.get("lookahead", timestep)
            self.stop_time = kwargs.get("stop_time", float("inf"))

            self.node = HLABaseNode(MorseBaseAmbassador, fom, node_name,
                                    federation, sync_point, sync_register, 
                                    self.time_sync, timestep, lookahead)
        except KeyError as error:
            logger.error("One of [fom, name, federation] attribute is not configured: "
                         "Cannot create HLADatastreamManager")
            raise

    def finalize(self):
        DatastreamManager.finalize(self)
        self.node.finalize()

    def register_component(self, component_name, component_instance, mw_data):
        """ Open the port used to communicate by the specified component.
        """

        mw_data[3]['__hla_node'] = self.node

        DatastreamManager.register_component(self, component_name,
                                                   component_instance, mw_data)

    def action(self):
        if self._time_initialized:
            self.node.morse_ambassador.advance_time()
        else:
            self.node.init_time()
            self._time_initialized = True
        if self.time_sync and \
            self.stop_time < self.node.morse_ambassador.logical_time:
            blenderapi.persistantstorage().serviceObjectDict["simulation"].quit()
