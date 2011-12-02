import logging; logger = logging.getLogger("morse." + __name__)
import socket
import select
import json
import morse.core.middleware
from functools import partial
from morse.core import services

class MorseSocketServ:
    def __init__(self, port, component_name):
        # List of socket clients
        self._client_sockets = []
        self._message_size = 1024
        self._component_name = component_name

        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind((str(socket.INADDR_ANY), port))
        self._server.listen(1)

        logger.info("Socket Mw Server now listening on port " + str(port) + \
                    " for component " + str(component_name) + ".")

    def __del__(self):
        """ Terminate the ports used to accept requests """
        if self._client_sockets:
            logger.info("Closing client sockets...")
            for s in self._client_sockets:
                s.close()

        if self._server:
            logger.info("Shutting down connections to server...")
            self._server.shutdown(socket.SHUT_RDWR)
            logger.info("Closing socket server...")
            self._server.close()
            del self._server

    def main_export(self, component_instance):
        sockets = self._client_sockets + [self._server]

        try:
            inputready, outputready, exceptready = select.select(sockets, sockets, [], 0)
        except select.error:
           pass
        except socket.error:
           pass

        if self._server in inputready:
            sock, addr = self._server.accept()
            self._client_sockets.append(sock)

        if outputready != []:
            message = (json.dumps(component_instance.local_data) + '\n').encode()
            for o in outputready:
                try:
                    o.send(message)
                except socket.error:
                    self.close_socket(o)

    def main_read(self, component_instance):
        sockets = self._client_sockets + [self._server]
        try:
            inputready, outputready, exceptready = select.select(sockets, [], [], 0)
        except select.error:
           pass
        except socket.error:
           pass

        for i in inputready:
            if i == self._server:
                sock, addr = self._server.accept()
                if self._client_sockets != []:
                    logger.warning("More than one clients for an actuator!!")
                self._client_sockets.append(sock)
            else:
                try:
                    msg = i.recv(self._message_size)
                    logger.debug("received msg %s" % msg)
                    if msg == b'':
                        self.close_socket(i)
                    else:
                        component_instance.local_data = json.loads(msg.decode('utf-8'))
                except socket.error as detail:
                    self.close_socket(i)

    def close_socket(self, sock):
        self._client_sockets.remove(sock)
        try:
            sock.close()
        except socket.error as error_info:
            logger.warning("Socket error catched while closing: " + str(error_info))


class MorseSocketClass(morse.core.middleware.MorseMiddlewareClass):
    """ External communication using sockets. """

    def __init__(self):
        """ Initialize the socket connections """
        # Call the constructor of the parent class
        super(self.__class__,self).__init__()

        # port -> MorseSocketServ
        self._server_dict = {}

        # component name (string)  -> Port (int)
        self._component_nameservice = {}

        self._base_port = 60000

        # Register two special services in the socket service manager:

        # TODO To use a new special component instead of 'simulation',
        # uncomment the line :-)
        # GameLogic.morse_services.register_request_manager_mapping("streams", "SocketRequestManager")
        services.do_service_registration(self.list_streams, 'simulation')
        services.do_service_registration(self.get_stream_port, 'simulation')
    
    def list_streams(self):
        return list(self._component_nameservice.keys())

    def get_stream_port(self, name):
        port = -1
        try:
            port = self._component_nameservice[name]
        except KeyError:
            pass

        return port

    def register_component(self, component_name, component_instance, mw_data):
        """ Open the port used to communicate by the specified component.
        """
        # Extract the information for this middleware
        # This will be tailored for each middleware according to its needs
        function_name = mw_data[1]
        serv = MorseSocketServ(self._base_port, component_name)
        self._server_dict[self._base_port] = serv
        self._component_nameservice[component_name] = self._base_port
        self._base_port = self._base_port + 1

        # Choose what to do, depending on the function being used
        # Data read functions
        if function_name == "read_message":
            component_instance.input_functions.append(partial(MorseSocketServ.main_read, serv))
        # Data write functions
        elif function_name == "post_message":
            component_instance.output_functions.append(partial(MorseSocketServ.main_export, serv))

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
