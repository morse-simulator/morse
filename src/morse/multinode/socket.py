import logging; logger = logging.getLogger("morse." + __name__)
import socket
import pickle
import mathutils

from morse.core import blenderapi
from morse.core.multinode import SimulationNodeClass

class SocketNode(SimulationNodeClass):
    """ 
    Implements multinode simulation using sockets.
    """

    node_socket = None
    connected = False
    out_data = {}

    def initialize(self):
        """
        Create the socket that will be used to commmunicate to the server.
        """
        logger.debug("Connecting to port %s:%d" % (self.host, self.port))
        try:
            self.node_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.node_socket.connect((self.host, self.port))
            logger.info("Connection established to node manager (%s, %s)" % (self.host, self.port))
            self.connected = True
        except socket.error as detail:
            logger.warning("Multi-node simulation not available!!")
            logger.info("\tUnable to connect to server: (%s, %s)" % (self.host, self.port))
            logger.info("\t%s" % detail)
            self.connected = False

    def _exchange_data(self, out_data):
        """ Send and receive pickled data through a socket """
        # Use the existing socket connection
        if self.connected:
            message = pickle.dumps([self.node_name, out_data])
            sock = self.node_socket
            sock.send(message)
            response = sock.recv(1024)
            in_data = pickle.loads(response)
            logger.debug("Received: %s" % in_data)
            return (in_data)

    def synchronize(self):
        if self.connected:
            # Get the coordinates of local robots
            robot_dict = blenderapi.persistantstorage().robotDict
            for obj, local_robot_data in robot_dict.items():
                #self.out_data[obj.name] = [obj.worldPosition.to_tuple()]
                euler_rotation = obj.worldOrientation.to_euler()
                self.out_data[obj.name] = [obj.worldPosition.to_tuple(), [euler_rotation.x, euler_rotation.y, euler_rotation.z]]
            # Send the encoded dictionary through a socket
            #  and receive a reply with any changes in the other nodes
            in_data = self._exchange_data(self.out_data)

            if in_data != None:
                scene = blenderapi.scene()
                # Update the positions of the external robots
                for obj_name, robot_data in in_data.items():
                    try:
                        obj = scene.objects[obj_name]
                        if obj not in robot_dict:
                            logger.debug("Data received: ", robot_data)
                            obj.worldPosition = robot_data[0]
                            obj.worldOrientation = mathutils.Euler(robot_data[1]).to_matrix()
                    except KeyError as detail:
                        logger.info("Robot %s not found in this simulation scenario, but present in another node. Ignoring it!" % detail)

    def finalize(self):
        """ Close the communication socket. """
        self.node_socket.close()
        self.connected = False
