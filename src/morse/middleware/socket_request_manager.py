import socket
import select

from morse.helpers.request_manager import RequestManager, MorseRPCInvokationError

SERVER_HOST = '' #all available interfaces
SERVER_PORT = 60001

class SocketRequestManager(RequestManager):
    """Implements services to control the MORSE simulator over
    raw ASCII sockets.

    The syntax of requests is:
    >>> component_name service [params in JSON format]

    The server answers:
    >>> OK|FAIL component_name service result_in_JSON|error_msg

    In case of failure while the request is being parsed, the
    server returns 'undefined' as component name and service name.
    """

    def __str__(self):
        return "Socket service manager"

    def _initialization(self):

        #Hold for each client socket a mapping to the socket file interface (as returned
        # by socket.makefile()
        self._client_sockets = {}
        
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        try:
            self._server.bind((SERVER_HOST, SERVER_PORT))
            self._server.listen(1)
        except socket.error as msg:
            self._server.close()
            self._server = None

        if not self._server:
            print("Couldn't bind the socket server! Aborting.")
            return False

        print("Socket service manager now listening on port " + str(SERVER_PORT) + ".")


        # This map holds the results (either success or failures)
        # to output to a client.
        # It maps a socket to a tuple of 4 items:
        # (success_status, component_name, service_name, result)
        # If success_status is False (failure), result contains the
        # error message (if available)
        self._results_to_output = {}

        # For long term request, this holds the mapping between a
        # service and the socket which requested it.
        self._pending_sockets = {}

        return True

    def _finalization(self):

        if self._client_sockets:
            print("Closing client sockets...")
            for s, f in self._client_sockets.items():
                f.close()
                s.close()

        if self._server:
            print("Closing socket server...")
            self._server.close()

        return True

    def _on_service_completion(self, component, service, result):

        s = None

        try:
            s = self._pending_sockets[component + "#" + service]
        except KeyError:
            print(str(self) + ": ERROR: I can not find the socket which requested " + request)
            return

        if s in self._results_to_output:
            self._results_to_output[s].append((True, component, service, results))
        else:
            self._results_to_output[s] = [(True, component, service, results)]

    def _post_registration(self, component, service, rpc, is_done):
        return True

    def _main(self):
        
        sockets = self._client_sockets.keys() + [self._server]

        try:
            inputready, outputready, exceptready = select.select(sockets, sockets, [])
        except select.error:
           pass
        except socket.error:
           pass
        
        if self._results_to_output:
            for o in outputready:
                if o in self._results_to_output:
                    for r in self._results_to_output[o]:
                        response = "%s %s %s %s" % ('OK' if r[0] else 'FAIL', r[1], r[2], str(r[3]))
                        print("Sending back " + response + " to " + str(o))
                        self._client_sockets[o].write(response)
                        self._client_sockets[o].write("\n")
                        self._client_sockets[o].flush()
                    del self._results_to_output[o]
        
        for i in inputready:
            if i == self._server:
                sock, addr = self._server.accept()
                self._client_sockets[sock] = sock.makefile() #convert the socket into a file interface to ease reading
                print("Accepted new service connection from " + str(addr))

            else:
                req = self._client_sockets[i].readline().strip()
                print("Got '" + req + "' from " + str(i))

                component = service = "undefined"

                try:

                    component, service, params = self.parse_request(req)

                    # _on_incoming_request returns either 
                    #(True, result) if it's a 'short term'
                    # request that has been immediately executed, or
                    # (False, None) if it's a long term request whose
                    # termination will be notified via _is_completed.
                    short_term, results = self._on_incoming_request(component, service, params)

                    if short_term:
                        if i in self._results_to_output:
                            self._results_to_output[i].append((True, component, service, results))
                        else:
                            self._results_to_output[i] = [(True, component, service, results)]
                    else:
                        # Stores the mapping service/socket to notify
                        # the right socket when the service completes.
                        # (cf :py:meth:_is_completed)
                        self._pending_sockets[component + "#" + service] = i


                except MorseRPCInvokationError as e:
                        if i in self._results_to_output:
                            self._results_to_output[i].append((False, component, service, e.value))
                        else:
                            self._results_to_output[i] = [(False, component, service, e.value)]




    def parse_request(self, req):
        """
        Parse the incoming request.
        """

        tokens = req.split(" ", 2)
        if len(tokens) < 2 or len(tokens) > 3:
            raise MorseRPCInvokationError("Invalid request: at least 2 values and at most 3 tokens are expected (component, service, [params]i)")

        if len(tokens) == 2:
            component, service = tokens
            p = None
        
        else:
            component, service, params = tokens

            try:
                p =  eval(params, {"__builtins__": None},{})
            except SyntaxError as se:
                raise MorseRPCInvokationError("Invalid request syntax: error while parsing the parameters.")

        return (component, service, p)
