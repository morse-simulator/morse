import logging; logger = logging.getLogger("morse." + __name__)
import socket
import select
import json

from morse.core.request_manager import RequestManager, MorseRPCInvokationError
from morse.core import status

SERVER_HOST = '' #all available interfaces
SERVER_PORT = 4000
MAX_TRIES = 10 # Number of alternative ports to try if the default is already busy

class SocketRequestManager(RequestManager):
    """Implements services to control the MORSE simulator over
    raw ASCII sockets.

    The syntax of requests is:

    >>> id component_name service [params with Python syntax]

    ``id`` is an identifier set by the client to conveniently identify
    the request. It must be less that 80 chars in [a-zA-Z0-9].

    The server answers:

    >>> id status result_in_python|error_msg

    ``status`` is one of the constants defined in :py:mod:`morse.core.status`.

    """

    def __str__(self):
        return "Socket service manager"

    def initialization(self):
        global SERVER_PORT
        self._client_sockets = []
        
        # For asynchronous request, this holds the mapping between a
        # request_id and the socket which requested it.
        self._pending_sockets = {}

        # Stores for each socket client the pending results to write
        # back.
        self._results_to_output = {}

        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        port_ok = False
        for i in range(MAX_TRIES):
            try:
                self._server.bind((SERVER_HOST, SERVER_PORT))
                self._server.listen(1)
                port_ok = True
                break
            except socket.error as msg:
                SERVER_PORT = SERVER_PORT + 1

        if not port_ok:
            logger.error("Couldn't bind the socket server! Port busy?")
            self._server.close()
            self._server = None
            return False

        logger.info("Socket service manager now listening on port " + str(SERVER_PORT) + ".")

        return True

    def finalization(self):
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

        return True


    def on_service_completion(self, request_id, results):

        s = None

        try:
            s, id = self._pending_sockets[request_id]
            del self._pending_sockets[request_id]
        except KeyError:
            logger.info(str(self) + ": ERROR: I can not find the socket which requested " + str(request_id))
            return

        if s in self._results_to_output:
            self._results_to_output[s].append((id, results))
        else:
            self._results_to_output[s] = [(id, results)]

    def post_registration(self, component, service, is_async):
        return True

    def main(self):
        
        sockets = self._client_sockets + [self._server]

        try:
            inputready, outputready, exceptready = select.select(sockets, sockets, [], 0) #timeout = 0 : Never block, just poll
        except select.error:
            pass
        except socket.error:
            pass

        for i in inputready:
            if i == self._server:
                sock, addr = self._server.accept()

                logger.info("Accepted new service connection from " + str(addr))
                self._client_sockets.append(sock)

            else:
                raw = i.recv(4096)
                if not raw:
                    # an empty read means that the remote host has
                    # disconnected itself
                    logger.debug("Socket closed by client! Closing it on my side.")
                    i.close()
                    self._client_sockets.remove(i)
                    continue
                raw = raw.decode()
                assert(raw[-1] == '\n')
                for req in raw[:-1].split('\n'):
                    req = req.strip()

                    component = service = "undefined"

                    try:
                        try:
                            id, req = req.split(None, 1)
                        except ValueError: # Request contains < 2 tokens.
                            id = req
                            raise MorseRPCInvokationError("Malformed request! ")

                        id = id.strip()

                        logger.info("Got '" + req + "' (id = " + id + ") from " + str(i))

                        if len(req.split()) == 1 and req in ["cancel"]:
                            # Aborting a running request!
                            for internal_id, user_id in self._pending_sockets.items():
                                if user_id[1] == id:
                                    self.abort_request(internal_id)

                        else:
                            component, service, params = self._parse_request(req)

                            # on_incoming_request returns either 
                            #(True, result) if it's a synchronous
                            # request that has been immediately executed, or
                            # (False, request_id) if it's an asynchronous request whose
                            # termination will be notified via
                            # on_service_completion.
                            is_sync, value = self.on_incoming_request(component, service, params)

                            if is_sync:
                                if i in self._results_to_output:
                                    self._results_to_output[i].append((id, value))
                                else:
                                    self._results_to_output[i] = [(id, value)]
                            else:
                                # Stores the mapping request/socket to notify
                                # the right socket when the service completes.
                                # (cf :py:meth:on_service_completion)
                                # Here, 'value' is the internal request id while
                                # 'id' is the id used by the socket client.
                                self._pending_sockets[value] = (i, id)


                    except MorseRPCInvokationError as e:
                        if i in self._results_to_output:
                            self._results_to_output[i].append((id, (status.FAILED, e.value)))
                        else:
                            self._results_to_output[i] = [(id, (status.FAILED, e.value))]

        if self._results_to_output:
            for o in outputready:
                if o in self._results_to_output:
                    for r in self._results_to_output[o]:
                        return_value = None
                        try:
                            if r[1][1]:
                                return_value = json.dumps(r[1][1])
                        except TypeError as te:
                            logger.error("Error while serializing a service return value to JSON!\n" +\
                                    "Details:" + te.value)
                        response = "%s %s%s" % (r[0], r[1][0], (" " + return_value) if return_value else "")
                        try:
                            o.send((response + "\n").encode())
                            logger.info("Sent back " + response + " to " + str(o))
                        except socket.error:
                            logger.warning("It seems that a socket client left while I was sending stuff to it. Closing the socket.")
                            o.close()
                            self._client_sockets.remove(o)

                    del self._results_to_output[o]


    def _parse_request(self, req):
        """
        Parse the incoming request.
        """

        tokens = req.split(None, 2)

        if len(tokens) < 1 or len(tokens) > 3:
            raise MorseRPCInvokationError("Malformed request: at least 3 values and at most 4 are expected (id, component, service, [params])")

        if len(tokens) == 2:
            component, service = tokens
            p = None
        else:
            component, service, params = tokens
            
            params = params.strip()

            if params:
                try:
                    p =  json.loads(params)
                except (NameError, SyntaxError, ValueError) as e:
                    raise MorseRPCInvokationError("Invalid request syntax: error while parsing the parameters: <%s>. %s" % (params, str(e)))
            else:
                p = None
        return (component, service, p)
