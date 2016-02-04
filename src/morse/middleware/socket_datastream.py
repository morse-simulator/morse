import logging; logger = logging.getLogger("morse." + __name__)
import socket
import select
import json
import errno
import time
from morse.core.datastream import DatastreamManager
from morse.helpers.transformation import Transformation3d
from morse.middleware import AbstractDatastream
from morse.core import services
from morse.core.exceptions import MorseRPCInvokationError, MorseMiddlewareError

try:
    import mathutils
except ImportError:
    # running outside Blender
    mathutils = None

class MorseEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, mathutils.Vector):
            return obj[:]
        if isinstance(obj, mathutils.Matrix):
            # obj[:][:] gives list(mathutils.Vector)
            return [list(vec) for vec in obj]
        if isinstance(obj, mathutils.Quaternion):
            return {'x' : obj.x, 'y': obj.y, 'z': obj.z, 'w': obj.w }
        if isinstance(obj, mathutils.Euler):
            return {'yaw': obj.z, 'pitch': obj.y, 'roll': obj.x }
        if isinstance(obj, Transformation3d):
            return {'x': obj.x, 'y': obj.y, 'z': obj.z,
                    'yaw': obj.yaw, 'pitch': obj.pitch, 'roll': obj.roll }
        return json.JSONEncoder.default(self, obj)

class SocketServ(AbstractDatastream):

    def initialize(self):
        # List of socket clients
        self._client_sockets = []
        self._message_size = 4096

        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind(('', self.kwargs['port']))
        self._server.listen(1)

        logger.info("Socket Mw Server now listening on port " + str(self.kwargs['port']) + \
                    " for component " + str(self.component_name) + ".")

    def finalize(self):
        """ Terminate the ports used to accept requests """
        if self._client_sockets:
            logger.info("Closing client sockets...")
            for s in self._client_sockets:
                s.close()

        if self._server:
            try:
                logger.info("Shutting down connections to server...")
                self._server.shutdown(socket.SHUT_RDWR)
            except socket.error as err_info:
                # ignore exception raised on OSX for closed sockets
                if err_info.errno != errno.ENOTCONN:
                    raise
            logger.info("Closing socket server...")
            self._server.close()

    def close_socket(self, sock):
        try:
            self._client_sockets.remove(sock)
            sock.close()
        except socket.error as error_info:
            logger.warning("Socket error catched while closing: " + str(error_info))
            del self._server
        except:
            pass

class SocketPublisher(SocketServ):

    _type_name = "straight JSON serialization"

    def default(self, ci='unused'):
        sockets = self._client_sockets + [self._server]

        try:
            inputready, outputready, _ = select.select(sockets, sockets, [], 0)
        except select.error:
            pass
        except socket.error:
            pass

        if self._server in inputready:
            sock, _ = self._server.accept()
            self._client_sockets.append(sock)

        if outputready:
            message = self.encode()
            for o in outputready:
                try:
                    o.send(message)
                except socket.error:
                    self.close_socket(o)

    def encode(self):
        js = json.dumps(self.component_instance.local_data, cls=MorseEncoder)
        return (js + '\n').encode()

class SocketReader(SocketServ):

    _type_name = "straight JSON deserialization"

    def default(self, ci='unused'):
        sockets = self._client_sockets + [self._server]
        try:
            inputready, outputready, exceptready = select.select(sockets, [], [], 0)
        except select.error:
            pass
        except socket.error:
            pass

        got_new_information = False

        for i in inputready:
            if i == self._server:
                sock, addr = self._server.accept()
                logger.debug("New client connected to %s datastream" % self.component_name)
                if self._client_sockets:
                    logger.warning("More than one client trying to write on %s datastream!!" % self.component_name)
                self._client_sockets.append(sock)
            else:
                try:
                    buf = []
                    msg = ""
                    full_msg = False
                    while not full_msg:
                        msg = i.recv(self._message_size).decode()
                        logger.debug("received msg %s" % msg)
                        if not msg: # client disconnected
                            self.close_socket(i)
                        else:
                            buf.append(msg)
                            full_msg = (len(msg) != self._message_size)
                    if not msg.endswith('\n'):
                        logger.error("Malformed message on socket datastream "+\
                                "(no linefeed at the end): <%s>" % msg)
                        continue
                    msg = ''.join(buf).rstrip("\n").split("\n")
                    logger.debug("received msg %s" % msg)
                    if len(msg)>1:
                        logger.warning("Messages missed on socket datastream! <%s>" % msg[:-1])
                    self.component_instance.local_data = self.decode(msg[-1]) # keep only the last msg if we got several in row
                    got_new_information = True
                except socket.error as detail:
                    self.close_socket(i)

        return got_new_information

    def decode(self, msg):
        return json.loads(msg)


class SocketDatastreamManager(DatastreamManager):
    """ External communication using sockets. """

    def __init__(self, args, kwargs):
        """ Initialize the socket connections """
        # Call the constructor of the parent class
        DatastreamManager.__init__(self, args, kwargs)

        self.time_sync = kwargs.get('time_sync', False)
        self.sync_port = kwargs.get('sync_port', 6000)

        if self.time_sync:
            self._init_trigger()

        # port -> MorseSocketServ
        self._server_dict = {}

        # component name (string)  -> Port (int)
        self._component_nameservice = {}

        # Base port
        self._base_port = 60000

        # Register two special services in the socket service manager:

        # TODO To use a new special component instead of 'simulation',
        # uncomment the line :-)
        # blenderapi.persistantstorage().morse_services.register_request_manager_mapping("streams", "SocketRequestManager")
        services.do_service_registration(self.list_streams, 'simulation')
        services.do_service_registration(self.get_stream_port, 'simulation')
        services.do_service_registration(self.get_all_stream_ports, 'simulation')

    def __del__(self):
        if self.time_sync:
            self._end_trigger()

    def _init_trigger(self):
        self._sync_client = None
        self._sync_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sync_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sync_server.bind(('', self.sync_port))
        self._sync_server.listen(1)
        logger.info("Creating clock synchronisation on port %d" % self.sync_port)

    def _wait_trigger(self):
        # If there is some client, just wait on it
        if self._sync_client:
            logger.debug("Waiting trigger")
            msg = self._sync_client.recv(2048)
            now = time.time()
            logger.debug('Synced after %f' % (now - self._last_sync_time))
            self._last_sync_time = now
            if not msg: #deconnection of client
                self._sync_client = None
        else:
            # Otherwise, we just check if there is some client waiting
            # If there is no client, we do not block for the moment to avoid
            # weird interaction at the startup
            logger.debug("Checking for some client on synchronisation port")
            try:
                inputready, _, _ = select.select([self._sync_server], [], [], 0)
            except select.error:
                pass
            except socket.error:
                pass

            if self._sync_server in inputready:
                self._sync_client, _ = self._sync_server.accept()
                self._last_sync_time = time.time()

    def _end_trigger(self):
        self._sync_client.close()
        self._sync_server.shutdown(socket.SHUT_RDWR)

    def list_streams(self):
        """ List all publish streams.
        """
        return list(self._component_nameservice.keys())

    def get_stream_port(self, name):
        """ Get stream port for stream name.
        """
        port = -1
        try:
            port = self._component_nameservice[name]
        except KeyError:
            pass
        
        if port < 0:
            raise MorseRPCInvokationError("Stream unavailable for component %s" % name)
        
        return port

    def get_all_stream_ports(self):
        """ Get stream ports for all streams.
        """
        return self._component_nameservice

    def register_component(self, component_name, component_instance, mw_data):
        """ Open the port used to communicate by the specified component.
        """
        register_success = False
        must_inc_base_port = False

        kwargs = mw_data[3]

        if not 'port' in kwargs:
            must_inc_base_port = True
            kwargs['port'] = self._base_port

        while not register_success:
            try:
                # Create a socket server for this component
                serv = DatastreamManager.register_component(self, component_name,
                                                 component_instance, mw_data)
                register_success = True
            except socket.error as error_info:
                if error_info.errno ==  errno.EADDRINUSE:
                    kwargs['port'] += 1
                    if must_inc_base_port:
                        self._base_port += 1
                else:
                    raise

        self._server_dict[kwargs['port']] = serv
        self._component_nameservice[component_name] = kwargs['port']
        if must_inc_base_port:
            self._base_port += 1

    def action(self):
        if self.time_sync:
            self._wait_trigger()
