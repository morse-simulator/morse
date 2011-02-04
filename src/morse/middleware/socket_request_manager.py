import socket
import select

from morse.helpers.request_manager import RequestManager, MorseRPCInvokationError

SERVER_HOST = '' #all available interfaces
SERVER_PORT = 60001

class SocketRequestManager(RequestManager):
    """Implements services to control the MORSE simulator over
    raw ASCII sockets.

    The syntax of requests is:
    >>> id component_name service [params with Python syntax]

    'id' is an identifier set by the client to conveniently identify
    the request. It must be less that 80 chars in [a-zA-Z0-9].

    The server answers:
    >>> id OK|FAIL result_in_python|error_msg

    """

    def __str__(self):
        return "Socket service manager"

    def _initialization(self):

        #Hold for each client socket a mapping to the socket file interface (as returned
        # by socket.makefile()
        self._client_sockets = {}
        
        # For asynchronous request, this holds the mapping between a
        # request_id and the socket which requested it.
        self._pending_sockets = {}

        # Stores for each socket client the pending results to write
        # back.
        self._results_to_output = {}

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

    def _on_service_completion(self, request_id, results):

        s = None

        try:
            s, id = self._pending_sockets[request_id]
        except KeyError:
            print(str(self) + ": ERROR: I can not find the socket which requested " + request)
            return

        if s in self._results_to_output:
            self._results_to_output[s].append((id, True, results))
        else:
            self._results_to_output[s] = [(id, True, results)]

    def _post_registration(self, component, service, rpc, is_async):
        return True

    def _main(self):
        
        sockets = self._client_sockets.keys() + [self._server]

        try:
            inputready, outputready, exceptready = select.select(sockets, sockets, [])
        except select.error:
           pass
        except socket.error:
           pass

        for i in inputready:
            if i == self._server:
                sock, addr = self._server.accept()
                self._client_sockets[sock] = sock.makefile() #convert the socket into a file interface to ease reading
                print("Accepted new service connection from " + str(addr))

            else:
                req = self._client_sockets[i].readline().strip()

                component = service = "undefined"

                try:
                    try:
                        id, req = req.split(" ", 1)
                    except ValueError: # Request contains < 2 tokens.
                        id = req
                        raise MorseRPCInvokationError("Malformed request! ")

                    id = id.strip()
                
                    print("Got '" + req + "' (id = " + id + ") from " + str(i))

                    component, service, params = self.parse_request(req)

                    # _on_incoming_request returns either 
                    #(True, result) if it's a synchronous
                    # request that has been immediately executed, or
                    # (False, request_id) if it's an asynchronous request whose
                    # termination will be notified via
                    # _on_service_completion.
                    is_sync, value = self._on_incoming_request(component, service, params)

                    if is_sync:
                        if i in self._results_to_output:
                            self._results_to_output[i].append((id, True, value))
                        else:
                            self._results_to_output[i] = [(id, True, value)]
                    else:
                        # Stores the mapping request/socket to notify
                        # the right socket when the service completes.
                        # (cf :py:meth:_on_service_completion)
                        # Here, 'value' is the internal request id while
                        # 'id' is the id used by the socket client.
                        self._pending_sockets[value] = (i, id)


                except MorseRPCInvokationError as e:
                        if i in self._results_to_output:
                            self._results_to_output[i].append((id, False, e.value))
                        else:
                            self._results_to_output[i] = [(id, False, e.value)]
        
        if self._results_to_output:
            for o in outputready:
                if o in self._results_to_output:
                    for r in self._results_to_output[o]:
                        response = "%s %s %s" % (r[0], 'OK' if r[1] else 'FAIL', str(r[2]))
                        print("Sending back " + response + " to " + str(o))
                        self._client_sockets[o].write(response)
                        self._client_sockets[o].write("\n")
                        self._client_sockets[o].flush()
                    del self._results_to_output[o]


    def parse_request(self, req):
        """
        Parse the incoming request.
        """

        tokens = req.split(" ", 2)
        if len(tokens) < 2 or len(tokens) > 3:
            raise MorseRPCInvokationError("Malformed request: at least 3 values and at most 4 are expected (id, component, service, [params])")

        if len(tokens) == 2:
            component, service = tokens
            p = None
        
        else:
            component, service, params = tokens

            try:
                p =  eval(params, {"__builtins__": None},{})
            except (NameError, SyntaxError) as e:
                raise MorseRPCInvokationError("Invalid request syntax: error while parsing the parameters. " + str(e))

        return (component, service, p)
