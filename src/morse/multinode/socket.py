import logging; logger = logging.getLogger("morse." + __name__)
import mathutils

from morse.core import blenderapi
from morse.core.multinode import SimulationNodeClass

from pymorse.stream import StreamJSON, PollThread

class SocketNode(SimulationNodeClass):
    """
    Implements multinode simulation using sockets.
    """

    out_data = {}

    def initialize(self):
        """
        Create the socket that will be used to commmunicate to the server.
        """
        self.node_stream = None
        self.poll_thread = None
        logger.debug("Connecting to %s:%d" % (self.host, self.port) )
        try:
            self.node_stream = StreamJSON(self.host, self.port)
            self.poll_thread = PollThread()
            self.poll_thread.start()
            if self.node_stream.connected:
                logger.info("Connected to %s:%s" % (self.host, self.port) )
        except Exception as err:
            logger.info("Multi-node simulation not available!")
            logger.warning("Unable to connect to %s:%s"%(self.host, self.port) )
            logger.warning(str(err))

    def _exchange_data(self, out_data):
        """ Send and receive pickled data through a socket """
        # Use the existing socket connection
        self.node_stream.publish([self.node_name, out_data])
        return self.node_stream.get(timeout=.1) or self.node_stream.last()

    def synchronize(self):
        if not self.node_stream:
            logger.debug("not self.node_stream")
            return
        if not self.node_stream.connected:
            logger.debug("not self.node_stream.connected")
            return

        # Get the coordinates of local robots
        for obj in blenderapi.persistantstorage().robotDict.keys():
            #self.out_data[obj.name] = [obj.worldPosition.to_tuple()]
            euler_rotation = obj.worldOrientation.to_euler()
            self.out_data[obj.name] = [obj.worldPosition.to_tuple(), \
                [euler_rotation.x, euler_rotation.y, euler_rotation.z]]
        # Send the encoded dictionary through a socket
        #  and receive a reply with any changes in the other nodes
        in_data = self._exchange_data(self.out_data)
        logger.debug("Received: %s" % in_data)

        if not in_data:
            return

        try:
            self.update_scene(in_data, blenderapi.scene())
        except Exception as e:
            logger.warning("error while processing incoming data: " + str(e))

    def update_scene(self, in_data, scene):
        # Update the positions of the external robots
        for obj_name, robot_data in in_data.items():
            try:
                obj = scene.objects[obj_name]
            except Exception as e:
                logger.debug("%s not found in this simulation scenario, but present in another node. Ignoring it!" % obj_name)
                continue
            if obj not in blenderapi.persistantstorage().robotDict:
                obj.worldPosition = robot_data[0]
                obj.worldOrientation = mathutils.Euler(robot_data[1]).to_matrix()

    def finalize(self):
        """ Close the communication socket. """
        if self.node_stream:
            self.node_stream.close()
            self.node_stream = None
        if self.poll_thread:
            self.poll_thread.syncstop(1)
            self.poll_thread = None
