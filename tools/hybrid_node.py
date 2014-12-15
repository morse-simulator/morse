#! /usr/bin/env python
"""
hybrid node
===========

Proxy between pocolibs and morse. Get real robot pose from picoweb and publish
it to morse multinode server (socket, not HLA).

usage: python hybrid_node.py mana|minnie [-d]
"""
import sys
import time
import urllib
import logging
from lxml import etree
from pymorse.stream import StreamJSON, PollThread

# initialize the logger
logger = logging.getLogger(__name__)
handler = logging.StreamHandler()
handler.setFormatter( logging.Formatter('[%(asctime)s][%(name)s][%(levelname)s] %(message)s') )
logger.addHandler( handler )
logger.setLevel(logging.INFO)

result = {
    'x': 0.0,
    'y': 0.0,
    'z': 0.0,
    'yaw':   0.0,
    'pitch': 0.0,
    'roll':  0.0,
}

def get_robot_pose(picoweb):
    xml = etree.parse(urllib.urlopen("%s/pom?get=Pos"%picoweb))
    res = xml.xpath("/pom/Pos/data/pomPos/mainToOrigin/euler")[0]
    return {key: float(res.xpath(key)[0].text) for key in result.keys()}

robot_picoweb = {
    'mana'  : 'http://140.93.16.55:8080',
    'minnie': 'http://140.93.16.57:8080',
    #'mana'  : 'http://mana-superbase:8080',
    #'minnie': 'http://minnie-base:8080',
    #'target': 'http://chrome-dreams:8080',
}

class HybridNode(object):
    """ Class definition for synchronisation of real robots with the MORSE simulator

    This component allows hybrid simulation, where the position of real robots
    is reflected in the simulator. Implemented using the socket multinode
    mechanism.
    """
    out_data = {}

    def __init__(self, host='localhost', port=65000, robot='mana'):
        """ Create the socket that will be used to commmunicate to the server. """
        self.robot = robot
        self.node_stream = None
        logger.debug("Connecting to %s:%d"%(host, port))
        try:
            self.node_stream = StreamJSON(host, port)
            self.poll_thread = PollThread()
            self.poll_thread.start()
            if self.node_stream.connected:
                logger.info("Connected to %s:%s" % (host, port))
        except Exception as e:
            logger.info("Multi-node simulation not available!")
            logger.warning("Unable to connect to %s:%s"%(host, port))
            logger.info(str(e))

    def _exchange_data(self):
        """ Send and receive pickled data through a socket """
        # Use the existing socket connection
        self.node_stream.publish(['hybrid', self.out_data])
        # Wait 1ms for incomming data or return the last one received
        return self.node_stream.get(timeout=.001) or self.node_stream.last()

    def synchronize(self):
        if not self.node_stream:
            logger.debug("not self.node_stream")
            return
        if not self.node_stream.connected:
            logger.debug("not self.node_stream.connected")
            return

        try:
            pose = get_robot_pose(robot_picoweb[self.robot])
            pose['z'] = 2 # XXX hack at laas
            self.out_data[self.robot] = [ (pose['x'], pose['y'], pose['z']),
                (pose['roll'], pose['pitch'], pose['yaw']) ]
            logger.info(repr(self.out_data))
        except Exception as e:
            logger.error(str(e))

        # Send the encoded dictionary through a socket
        #  and receive a reply with any changes in the other nodes
        in_data = self._exchange_data()
        logger.debug("Received: %s" % repr(in_data))

    def __del__(self):
        """ Close the communication socket. """
        if self.node_stream:
            self.node_stream.close()
        # asyncore.close_all() # make sure all connection are closed
        if 'poll_thread' in dir(self):
            self.poll_thread.syncstop()

def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 1
    if '-d' in argv[1:]:
        logger.setLevel(logging.DEBUG)

    logger.debug("Hybrid node started...")
    hn = HybridNode(robot=argv[1])

    try:
        while 1:
            hn.synchronize()
            time.sleep(0.5)
    except KeyboardInterrupt:
        logger.info("Quit (Ctrl+C)")
    finally:
        del hn
        logger.info("Closing all connections")

    logger.info("Bye!")
    return 0

if __name__ == '__main__':
    import sys
    sys.exit(main(sys.argv))
