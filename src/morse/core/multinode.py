import socket
import pickle
import mathutils

class SimulationNodeClass (object):
    """ Class defining the behaviour of a simulation node

    It will be used to synchronise the simulation when done
    on two or more Blender instances (simulation nodes)
    """

    node_name = "single node"
    node_socket = None
    host = "localhost"
    port = 65000

    out_data = {}

    def __init__(self, name="single node", server_address="localhost", server_port=65000):
        self.node_name = name
        self.host = server_address
        self.port = server_port
        self._init_socket()


    def __del__(self):
        self.finish_node()

    def _init_socket(self):
        """ Create the socket that will be used to commmunicate to the server
        """
        print ("Connecting to port %s:%d" % (self.host, self.port))
        try:
            self.node_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.node_socket.connect((self.host, self.port))
            return True
        except socket.error as detail:
            print ("Unable to connect to server: ", detail)
            return False


    def _exchange_data(self, out_data):
        """ Send and receive pickled data through a socket """
        if self._init_socket():
            message = pickle.dumps([self.node_name, out_data])
            sock = self.node_socket
            sock.send(message)
            response = sock.recv(1024)
            in_data = pickle.loads(response)
            #print("Received: %s" % in_data)
            self.finish_node()

            return (in_data)
        else:
            return None


    def synchronise_world(self, GameLogic):
        """ Share the data between multiple simulation nodes.

        Transmit the position and orientation of robots simulated in this node,
        and then receive the new localisation of any robot simulated in
        another instance of Blender.
        """
        # Get the coordinates of local robots
        for obj, local_robot_data in GameLogic.robotDict.items():
            #self.out_data[obj.name] = [obj.worldPosition.to_tuple()]
            euler_rotation = obj.worldOrientation.to_euler()
            self.out_data[obj.name] = [obj.worldPosition.to_tuple(), [euler_rotation.x, euler_rotation.y, euler_rotation.z]]
        # Send the encoded dictionary through a socket
        #  and receive a reply with any changes in the other nodes
        in_data = self._exchange_data(self.out_data)

        if in_data != None:
            scene = GameLogic.getCurrentScene()

            # Update the positions of the external robots
            for obj_name, robot_data in in_data.items():
                obj = scene.objects[obj_name]
                if obj not in GameLogic.robotDict:
                    #print ("Data received: ", robot_data)
                    obj.worldPosition = robot_data[0]
                    obj.worldOrientation = mathutils.Euler(robot_data[1]).to_matrix()


    def finish_node(self):
        """ Close the communication socket """
        self.node_socket.close()
