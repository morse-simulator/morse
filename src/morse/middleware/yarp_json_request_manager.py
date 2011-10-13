import logging; logger = logging.getLogger("morse." + __name__)
import sys
import yarp
import json
from collections import OrderedDict

from morse.core.request_manager import RequestManager, MorseRPCInvokationError
from morse.core import status


class YarpRequestManager(RequestManager):
    """Implements services to control the MORSE simulator over YARP

    The syntax of requests is:
    >>> id component_name service [params with Python syntax]

    'id' is an identifier set by the client to conveniently identify
    the request. It must be less that 80 chars in [a-zA-Z0-9].

    The server answers:
    >>> id OK|FAIL result_in_python|error_msg

    """

    def __str__(self):
        return "Yarp service manager"

    def initialization(self):

        # Create dictionaries for the input and output ports
        self._yarp_request_ports = dict()
        self._yarp_reply_ports = dict()
        # Create a dictionary for the port names
        self._component_ports = dict()

        # For asynchronous request, this holds the mapping between a
        # request_id and the socket which requested it.
        self._pending_ports = dict()

        # Stores for each port the pending results to write back.
        self._results_to_output = dict()

        # Create a dictionary for the evailable bottles
        self._in_bottles = dict()
        self._reply_bottles = dict()

        return True


    def finalization(self):
        logger.info("Closing yarp request ports...")
        for port in self._yarp_request_ports.values():
            port.close()

        return True


    def on_service_completion(self, request_id, results):
        port = None

        try:
            port, id = self._pending_ports[request_id]
        except KeyError:
            logger.info(str(self) + ": ERROR: I can not find the port which requested " + request_id)
            return

        if port in self._results_to_output:
            self._results_to_output[port].append((id, results))
        else:
            self._results_to_output[port] = [(id, results)]


    def post_registration(self, component_name, service, is_async):
        """ Register a connection of a service with YARP """
        # Get the Network attribute of yarp,
        #  then call its init method
        self._yarp_module = sys.modules['yarp']
        self.yarp_object = self._yarp_module.Network()

        # Create the names of the ports
        request_port_name = '/ors/services/{0}/request'.format(component_name)
        reply_port_name = '/ors/services/{0}/reply'.format(component_name)

        if not component_name in self._yarp_request_ports.keys():
            # Create the ports to accept and reply to requests
            request_port = self._yarp_module.BufferedPortBottle()
            reply_port = self._yarp_module.BufferedPortBottle()
            request_port.open(request_port_name)
            reply_port.open(reply_port_name)
            self._yarp_request_ports[component_name] = request_port
            self._yarp_reply_ports[component_name] = reply_port
        
            # Create bottles to use in the responses
            bottle_in = self._yarp_module.Bottle()
            self._in_bottles[component_name] = bottle_in
            bottle_reply = self._yarp_module.Bottle()
            self._reply_bottles[component_name] = bottle_reply

            logger.info("Yarp service manager now listening on port " + request_port_name + ".")
            logger.info("Yarp service manager will reply on port " + reply_port_name + ".")

        return True


    def main(self):
        """ Read commands from the ports, and prepare the response""" 
        # Read data from available ports
        for component_name, port in self._yarp_request_ports.items():
            # Get the bottles to read and write
            bottle_in = self._in_bottles[component_name] 
            bottle_reply = self._reply_bottles[component_name] 
            bottle_in = port.read(False)
            if bottle_in != None:
                logger.debug("Received command from port '%s'" % (component_name))
                id = 'unknown'

                try:
                    try:
                        id, component_name, service, params = self._parse_request(bottle_in)
                    except ValueError: # Request contains < 2 tokens.
                        raise MorseRPCInvokationError("Malformed request!")

                    logger.info("Got '%s | %s | %s' (id = %s) from %s" % (component_name, service, params, id, component_name))

                    # on_incoming_request returns either 
                    # (True, result) if it's a synchronous
                    # request that has been immediately executed, or
                    # (False, request_id) if it's an asynchronous request whose
                    # termination will be notified via
                    # on_service_completion.
                    is_sync, value = self.on_incoming_request(component_name, service, params)

                    if is_sync:
                        if port in self._results_to_output:
                            self._results_to_output[port].append((id, value))
                        else:
                            self._results_to_output[port] = [(id, value)]
                    else:
                        # Stores the mapping request/socket to notify
                        # the right port when the service completes.
                        # (cf :py:meth:on_service_completion)
                        # Here, 'value' is the internal request id while
                        # 'id' is the id used by the socket client.
                        self._pending_ports[value] = (port, id)


                except MorseRPCInvokationError as e:
                    if port in self._results_to_output:
                        self._results_to_output[port].append((id, (status.FAILED, e.value)))
                    else:
                        self._results_to_output[port] = [(id, (status.FAILED, e.value))]
        
        if self._results_to_output:
            for component_name, port in self._yarp_request_ports.items():
                if port in self._results_to_output:
                    for r in self._results_to_output[port]:
                        response = OrderedDict([
                            ('id', r[0]),
                            ('status', r[1][0]),
                            ('reply', (r[1][1] if r[1][1] else "")) ])
                            #('reply', "%s" % str(r[1][1]) if r[1][1] else "") ])
                        json_response = json.dumps(response)
                        # Send the reply through the same yarp port
                        reply_port = self._yarp_reply_ports[component_name]
                        bottle_reply = reply_port.prepare()
                        bottle_reply.clear()
                        bottle_reply.addString(json_response)
                        reply_port.write()
                        logger.debug("Sent back '" + str(response) + "'. Component: " + component_name + ". Port: " + str(port))
                            
                    del self._results_to_output[port]


    def _parse_request(self, bottle):
        """
        Parse the incoming bottle.
        """
        try:
            request_msg = bottle.get(0).toString()
            request = json.loads(request_msg, object_pairs_hook=OrderedDict)
        except (IndexError, ValueError) as e:
            raise MorseRPCInvokationError('Malformed request: expected a json econded request with this format: \'{id:13, component:Motion_Controller, service:goto, params:[5, 5, 0]}\' (all values enclosed in strings)')

        id = request['id']
        component_name = request['component']
        service = request['service']
        try:
            params = request['params']
            import ast
            p =  ast.literal_eval(params)
        except (NameError, SyntaxError) as e:
            raise MorseRPCInvokationError("Invalid request syntax: error while parsing the parameters. " + str(e))

        return (id, component_name, service, p)
