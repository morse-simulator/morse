from abc import ABCMeta, abstractmethod
import morse.helpers.transformation
import morse.core.services
from collections import OrderedDict

from morse.core.exceptions import MorseRPCInvokationError

class MorseObjectClass(object):
    """ Basic Class for all 3D objects (components) used in the simulation.
        Provides common attributes. """

    # Make this an abstract class
    __metaclass__ = ABCMeta

    def __init__ (self, obj, parent=None):
        """ Constructor method. """
        # Fill in the data sent as parameters
        self.blender_obj = obj
        self.robot_parent = parent

        self.on_completion = None
        """ When a task is considered 'completed' (the semantic of
        'completed' is left to each component), the default_action
        method is expected to call this callback (if not None) with
        the task status (from ``core.status.*``) + optional return value
        as a tuple.

        For instance::

          self.on_completion((status.FAILED, "Couldn't reach the target"))
          self.on_completion((status.SUCCESS, {'x':1.0, 'y':0.54}))
          self.on_completion((status.SUCCESS))
        """

        # Define the position of sensors with respect
        #  to their robot parent
        # TODO: implement this using morse.helpers.transformation
        if not parent == None:
            self.relative_position = obj.getVectTo(parent.blender_obj)

        # Create an instance of the 3d transformation class
        self.position_3d = morse.helpers.transformation.Transformation3d(obj)

        #: Dictionary to store the data used by each component
        self.local_data = OrderedDict()

        # Define lists of dynamically added functions
        self.del_functions = []
        

    def __del__(self):
        """ Destructor method. """
        print ("%s: I'm dying!!" % self.blender_obj.name)
        # Call specific functions added to this object
        for function in self.del_functions:
            function(self)


    #@abstractmethod
    def action(self):
        """ Call the regular action function of the component.
        
        Can be redefined in some of the subclases (sensor and actuator).
        """
        self.default_action()


    @abstractmethod
    def default_action(self):
        """ Base action performed by any object.

        This method should be implemented by all subclasses that
        will be instanced (GPS, v_Omega, ATRV, etc.).
        """
        pass


    def register_services(self):
        """
        Register the component services, if any.
        Methods to register are marked '_morse_service' by the '@service' decorator.
        """
        for fn in [getattr(self, fn) for fn in dir(self) if hasattr(getattr(self, fn), "_morse_service")]:
            name = fn._morse_service_name if fn._morse_service_name else fn.__name__
            morse.core.services.do_service_registration(fn, self.blender_obj.name, name, fn._morse_service_is_async)


    def completed(self, status, result = None):
        """ This method must be invoked by the component when a service completes.

        Calling this method triggers the notification of task completion to the client.

        :param morse.core.status status: status (success, failure...) of the task

        :param result: results of the service, if any (may be any valid Python object)

        """
        if self.on_completion:
             self.on_completion((status, result))
             self.on_completion = None

    def set_service_callback(self, cb):
        """ Sets the callback function that is to be invoked when the current
        request completes.

        This is automatically set by the @async_service decorator and should
        not usually be directly called.
        """
        if self.on_completion:
            import morse.core.status
            self.on_completion((morse.core.status.PREEMPTED, "New request received"))
            #raise MorseRPCInvokationError("A request is already ongoing")

        self.on_completion = cb
 

    def print_data(self):
        """ Print the current data for the component instance. """
        for variable, data in self.local_data.items():
            res = variable + str(data) + " "
        print ("%s" % res)
