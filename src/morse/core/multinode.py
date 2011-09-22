import logging; logger = logging.getLogger("morse." + __name__)
import os
from abc import ABCMeta, abstractmethod

class SimulationNodeClass (object):
    """ Class defining the behaviour of a simulation node.

    It will be used to synchronise the simulation when done
    on two or more simulation nodes.
    """

    # Make this an abstract class
    __metaclass__ = ABCMeta
    
    def __init__(self, name=os.uname()[1], server_address="localhost", server_port=65000):
        self.node_name = name
        self.host = server_address
        self.port = server_port
        self.initialize()

    def __del__(self):
        self.finalize()

    @abstractmethod
    def initialize():
        """ 
        Initialize the MORSE node.
        """
        pass

    @abstractmethod
    def synchronize(self, GameLogic):
        """
        Synchronize simulation nodes.
        Publishes node's robots to the other simulation nodes and
        update node's external robots from data published by other simulation
        nodes.
        """
        pass

    @abstractmethod
    def finalize(self):
        """
        Finalize the MORSE node.
        """
        pass
