import logging; logger = logging.getLogger("morse." + __name__)
logger.setLevel(logging.DEBUG)
import socket
import select

from morse.core.exceptions import *
from morse.core.request_manager import RequestManager
from morse.core import status

class PocolibsRequestManager(RequestManager):
    """Implements Pocolibs requests to control the MORSE simulator.
    
    This is done by re-implementing (parts of) the TCLserv protocol.
    """
    
    HOST = '' #all available interfaces
    PORT = 9473 # TCL serv port + 1

    def __str__(self):
        return "Pocolibs service manager"

    def initialization(self):
        
        #  socket -> identifier (int)
        self._clients = {} 

        # socket cmd -> socket answer (may be the same)
        self._answer_clients = {}  
        
        # Clients that have pending (ie asynchronous) requests
        self._pending_requests = {} 
        self._next_client_id = 0
        self._next_rqst_id = 0
        
        self._landing_request = None
        
        # Holds the output queue as a dictionary {socket:[message...]}
        self._results_to_output = {}

        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        try:
            self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._server.bind((PocolibsRequestManager.HOST, PocolibsRequestManager.PORT))
            self._server.listen(1)
        except socket.error as msg:
            self._server.close()
            self._server = None
            
        if not self._server:
            logger.info('Could not bind the Pocolibs server. Port busy?')
            return False

        logger.info("Pocolibs request manager now listening on port " + str(PocolibsRequestManager.PORT) + ".")
        
        # Prepare 'select'
        self._inputs = [self._server]
        self._outputs = []
        
        self._quitting = False
        
        return True

    def finalization(self):
        """ Terminate the ports used to accept requests """
        if self._clients:
            logger.info("Closing client sockets...")
            for s in self._clients.keys():
                s.shutdown(socket.SHUT_RDWR)
        
        if self._server:
            logger.info("Closing Pocolibs server...")
            self._server.shutdown(socket.SHUT_RDWR)
            del self._server

        return True


    def on_service_completion(self, intern_rqst_id, result):
        
        state, value = result
        
        s = None
        
        try:
            s, rqst_id, fqn = self._pending_requests[intern_rqst_id]
        except KeyError:
            logger.info(str(self) + ": ERROR: I can not find the socket which requested " + \
                  intern_rqst_id + ". Skipping it.")
            return

        res = str(rqst_id) + " " + fqn + \
                " TERM " + \
                ("OK" if state == status.SUCCESS else "") + \
                (" " + " ".join([str(i) for i in value]) if value else "")
        self._results_to_output.setdefault(self._answer_clients[s], []).append(res)
        

    def post_registration(self, component, service, is_async):
        return True # No post-registration steps

    def main(self):
        
        try:
            inputready, outputready, exceptready = select.select(self._inputs, self._outputs, [], 0) #timeout = 0 : Never block, just poll
        except select.error:
           pass
        except socket.error:
           pass

        for i in inputready:
            if i == self._server:
                conn, addr = self._server.accept()
                
                data = ""
                data = conn.recv(1024).decode('ascii')
                
                data = data.strip("\n\r")
                
                if data == "HELLO":
                    conn.send(("HELLO " + str(self._next_client_id) + "\r\n").encode('ascii'))
                    logger.info('Pocolibs request manager: new connection from %s', str(addr) )
                    self._clients[conn] = self._next_client_id
                    self._answer_clients[conn] = conn
                    self._inputs.append(conn)
                    self._outputs.append(conn)
                    self._next_client_id += 1
                else:
                    logger.info("Incorrect connection attempt - was expecting 'HELLO\\n', got '" + data + "'.")
                    conn.close()

            else:
                data = None
                try:
                    data = i.recv(1024).decode('ascii')
                except socket.error:
                    logger.warning("Client " + str(self._clients[i]) + " disconnected")
                    del self._clients[i]
                    self._inputs.remove(i)
                    self._outputs.remove(i)

                if not data:
                        return
                logger.debug("[Client " + str(self._clients[i]) + "]: " + data)
                
                        
                if data.startswith("BYE"):
                    logger.info("Client " + str(self._clients[i]) + " is leaving. Bye bye")
                    del self._clients[i]
                    self._inputs.remove(i)
                    self._outputs.remove(i)
                    continue
                    
                ok, msg = self._dispatch(data, self._clients[i])
        
                if ok:
                    self._results_to_output.setdefault(i, []).append("OK " + msg)
                else:
                    self._results_to_output.setdefault(i, []).append("ERROR " + msg)
                
                # After dispatching, we have a new, 'landing', request. Let's deal
                # with it.
                if self._landing_request:
                    
                    rqst_id, fqn, is_sync, result = self._landing_request
                    
                    if is_sync:
                        state, value = result
                        res = str(rqst_id) + " " + fqn + \
                            " TERM " + \
                            ("OK" if state == status.SUCCESS else "") + \
                            (" " + " ".join([str(i) for i in value]) if value else "")
                        self._results_to_output.setdefault(self._answer_clients[i], []).append(res)
                    else:
                        activity_id = 0 #TODO: For now, we do not support more than one instance of the same request
                        res = str(rqst_id) + " " + fqn + " ACK " + str(activity_id)
                        self._results_to_output.setdefault(self._answer_clients[i], []).append(res)
                        
                        # Stores the mapping request/socket to notify
                        # the right client when the service completes.
                        # (cf :py:meth:on_service_completion)
                        # Here, 'result' is the internal request id while
                        # 'rqst_id' is the id used by the socket client.
                        self._pending_requests[result] = (i, rqst_id, fqn)
                        
                    self._landing_request = None
        
        if self._results_to_output:
            for o in outputready:
                if o in self._results_to_output:
                    for r in self._results_to_output[o]:
                        try:
                            o.send((r + "\r\n").encode('ascii'))
                            logger.debug("Sending [" + r + "] to " + str(self._clients[o]))
                        except socket.error:
                            logger.info("It seems that a socket client left. Closing the socket.")
                            del self._clients[o]
                            self._inputs.remove(o)
                            self._outputs.remove(o)
                            
                    del self._results_to_output[o]


    def _dispatch(self, raw, client_id):
        """ This method implements the TCLserv protocol.
        
        :param raw: the raw string, as read on the socket
        :return: a tuple (status, message). status == True means the request
                 has been successfully dispatched. In that case, and if the 
                 request is a RQST command, 'message' contains the request id 
                 that will be used in subsequent ACK or TERM messages.
                 In other cases, 'message' contains the return value of the 
                 request (and can be empty).
                 status == False means that the disptaching failed. 'message'
                 contains the error message.
        """
        
        req = raw.split()        
        cmd = req[0]
        
        if cmd  == "REPLYTO":
            if client_id != int(req[1]):
                s_client = None
                s_reply = None
                for s, id in self._clients.items():
                    if id == client_id:
                        s_client = s
                    elif id == int(req[1]):
                        s_reply = s

                if (s_client and s_reply):
                    self._answer_clients[s_client] = s_reply

            return (True, str(client_id))
        
        if cmd == "RQST":
            component, rqst = req[1].strip("::").split("::")
            params = req[2:]
            logger.info("Incoming pocolibs request " + rqst + 
                    " for component " + component + 
                    " with params " + str(params))
            
            rqst_id = self._next_rqst_id
            self._next_rqst_id += 1
            
            try:
                # on_incoming_request returns either 
                #(True, result) if it's a synchronous
                # request that has been immediately executed, or
                # (False, request_id) if it's an asynchronous request whose
                # termination will be notified via
                # on_service_completion.
                # _landing_request will containt:
                #    - the request id
                #    - the request TCL 'full qualified name' (eg ::component::meth)
                #    - the 'is synchronous' flag
                #    - a value, as explained above
                self._landing_request = (rqst_id,"::" + component + "::" + rqst) + \
                                        self.on_incoming_request(component, rqst, params)
                
            except MorseMethodNotFoundError:
                # Request not found for module
                return (False, "1 invalid command name \\\"" + rqst + "Send\\\"")
            except MorseRPCNbArgsError:
                # Wrong # of args
                return (False, "1 wrong # args")
            except MorseRPCTypeError:
                # Wrong # of args
                return (False, "1 wrong type in args")

            return (True, str(rqst_id))
            
        if cmd == "cs::lsmbox": # want to list available modules
            return (True, " ".join(self.services().keys()))
        
        if cmd == "info":
            if req[1] == "commands":
                # The client want to retrieve the list of available requests.
                # Rebuild a 'TCL like' list of methods for the required module.
                component = req[2].split("::")[0]
                return(True, " ".join(["::" + component + "::" + method + "Send" for method in self.services()[component]]))
                
        if cmd in ["LM", "cs::init", "exec", "modules::connect", "ACK"]:
            # Not needed in simulation
            return (True, "")
        
        logger.error("command " + cmd + " not implemented!")
        return (False, "0 command " + cmd + " not implemented!")

