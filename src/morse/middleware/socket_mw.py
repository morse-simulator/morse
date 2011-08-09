import logging; logger = logging.getLogger("morse." + __name__)
import socket
import pickle
import morse.core.middleware

class MorseSocketClass(morse.core.middleware.MorseMiddlewareClass):
    """ External communication using sockets. """

    def __init__(self, obj, parent=None):
        """ Initialize the socket connections """
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self._socket_dict = dict()
        self._socket_ports = []
        self._socket_clients = dict()
        self._host = ''
        self._base_port = 60000
        self._message_size = 1024
        self._connected = False

    def register_component(self, component_name, component_instance, mw_data):
        """ Open the port used to communicate by the specified component.

        The name of the port is postfixed with in/out, according to
        the direction of the communication.
        """
        # Compose the name of the port
        parent_name = component_instance.robot_parent.blender_obj.name

        # Extract the information for this middleware
        # This will be tailored for each middleware according to its needs
        function_name = mw_data[1]

        try:
            # Get the reference to the function
            function = getattr(self, function_name)
        except AttributeError as detail:
            logger.error("%s. Check the 'component_config.py' file for typos" % detail)
            return

        #self.open_TCP_server(parent_name)
        self.open_UDP_server(parent_name)

        # Choose what to do, depending on the function being used
        # Data read functions
        if function_name == "read_message":
            #self.open_UDP_server(component_name, component_instance)
            component_instance.input_functions.append(function)
        # Data write functions
        elif function_name == "post_message":
            #self.open_UDP_server(component_name, component_instance)
            component_instance.output_functions.append(function)

        """
        # Data read functions
        elif function_name == "read_tcp_message":
            component_instance.input_functions.append(function)
        # Data write functions
        elif function_name == "post_tcp_message":
            component_instance.output_functions.append(function)
        # Image write functions
        elif function_name == "post_image_RGBA":
            self.open_UDP_server(component_name)
            component_instance.output_functions.append(function)
        """


    def open_UDP_server(self, robot_name):
        """ Create an UDP server, given a list of names. """
        if robot_name not in self._socket_dict:
            try:
                new_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            except socket.error as detail:
                logger.error("Unable to create socket: '{0}'".format(detail))
                return

            # Get the port number to use from the list
            # If none have been used, use a fixed number
            if self._socket_ports == []:
                socket_port = self._base_port
            # Otherwise, take the last port number, plus one
            else:
                socket_port = self._socket_ports[-1] + 1

            new_socket.bind((self._host, socket_port))
            new_socket.setblocking(0)

            # Register the port in the dictionary
            #  using the name of the parent as index
            self._socket_dict[robot_name] = new_socket
            self._socket_ports.append(socket_port)
            #self._socket_clients[robot_name] = host

            logger.info("Socket MW: Adding UDP socket to robot {0} on port {1}".format(robot_name, socket_port))

        else:
            pass
            #raise NameError("A port already exists for robot %s" % robot_name)



    def open_TCP_server(self, robot_name):
        """ Create an UDP server, given a list of names. """
        if robot_name not in self._socket_dict:
            try:
                new_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            except socket.error as detail:
                logger.error("Unable to create socket: '{0}'".format(detail))
                return

            # Get the port number to use from the list
            # If none have been used, use a fixed number
            if self._socket_ports == []:
                socket_port = self._base_port
            # Otherwise, take the last port number, plus one
            else:
                socket_port = self._socket_ports[-1] + 1

            new_socket.bind((self._host, socket_port))
            new_socket.listen(2)

            # Register the port in the dictionary
            #  using the name of the parent as index
            self._socket_dict[robot_name] = new_socket
            self._socket_ports.append(socket_port)
            #self._socket_clients[robot_name] = host

            logger.info("Socket MW: Adding TCP socket to robot {0} on port {1}".format(robot_name, socket_port))

        else:
            pass
            #raise NameError("A port already exists for robot %s" % robot_name)



    def finalize(self):
        """ Closes the ports and release the network."""
        for ors_socket in self._socket_dict.values():
            ors_socket.close()
        logger.info('Socket Mid: sockets have been closed.')
    

    def read_message(self, component_instance):
        """ Read a port expecting the data specified in data_types.

        The data is expected to be of a specified length.
        Retuns True after reading data, or False if no data received. """
        # Get the reference to the correct socket
        parent_name = component_instance.robot_parent.blender_obj.name
        in_socket = self._socket_dict[parent_name]

        try:
            message_data, client_addr = in_socket.recvfrom(self._message_size)
        except socket.error as detail:
            #logger.error("%s" % detail)
            return False

        # Store the socket returned so it can be used for writing
        self._socket_clients[component_instance.robot_parent] = client_addr
        logger.info("Connection received from client '{0}'".format(client_addr))
        pickled_data = pickle.loads(message_data)

        # Extract the values from the socket data
        i = 0
        for variable, data in component_instance.local_data.items():
            msg_data = pickled_data[i]
            component_instance.local_data[variable] = msg_data
            i = i + 1

        return True


    def post_message(self, component_instance):
        """ Send a message using a port."""
        parent_name = component_instance.robot_parent.blender_obj.name
        out_socket = self._socket_dict[parent_name]

        try:
            # Check that a connection has already been established
            #  for this robot
            client_addr = self._socket_clients[component_instance.robot_parent]
        except KeyError as detail:
            return

        message = pickle.dumps((component_instance.blender_obj.name, component_instance.local_data))

        logger.debug("Socket Mid: Send: '{0}' to host '{1}'".format(message, host))
        out_socket.sendto(message, client_addr)


    ### vvv NOT USED vvv ###
    def read_tcp_message(self, component_instance):
        """ Read a port expecting the data specified in data_types.

        The data is expected to be of a specified length.
        Retuns True after reading data, or False if no data received. """
        # Get the reference to the correct socket
        parent_name = component_instance.robot_parent.blender_obj.name
        in_socket = self._socket_dict[parent_name]

        # Check for connections
        if not self._connected:
            logger.info("Waiting for a connection")
            client_socket, address = in_socket.accept()
            if client_socket:
                self._socket_clients[component_instance.robot_parent] = client_socket
                logger.info("Client '{0}' connected to port {1}".format(address, in_socket))
                self._connected = True

        # If the connection is already established
        if self._connected:
            try:
                message_data = in_socket.recv(self._message_size)
            except socket.error as detail:
                #logger.error("%s" % detail)
                return False

            # Extract the values from the socket data
            pickled_data = pickle.loads(message_data)
            i = 0
            for variable, data in component_instance.local_data.items():
                msg_data = pickled_data[i]
                component_instance.local_data[variable] = msg_data
                i = i + 1

            return True


    def post_tcp_message(self, component_instance):
        """ Send a message using a port."""
        parent_name = component_instance.robot_parent.blender_obj.name
        out_socket = self._socket_dict[parent_name]

        try:
            # Check that a connection has already been established
            #  for this robot
            client_socket = self._socket_clients[component_instance.robot_parent]
        except KeyError as detail:
            return

        message = pickle.dumps((component_instance.blender_obj.name, component_instance.local_data))

        logger.debug("Socket Mid: Send: '{0}' to host '{1}'".format(message, host))
        out_socket.send(message)


    def post_image(self, component_instance):
        """ Send an RGB image using a port. (STILL INCOMPLETE)"""
        try:
            out_socket = self._socket_dict[component_instance.blender_obj.name]
            
            (conn, addr) = out_socket.accept()
            logger.info('Socket Mid: Connected by', addr)
            out_socket.send(message)

        except KeyError as detail:
            logger.error("Specified port does not exist: ", detail)
    ### ^^^ NOT USED ^^^ ###


    def print_open_sockets(self):
        """ Display a list of all currently opened sockets."""
        logger.info("Socket Mid: Currently opened sockets:")
        for name, socket in self._socket_dict.iteritems():
            logger.info(" - Port name '{0}' = '{1}'".format(name, socket))
