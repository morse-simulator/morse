import logging; logger = logging.getLogger("morse." + __name__)
import sys
import re
import yarp
import array
import morse.core.middleware
import GameLogic

class MorseYarpClass(morse.core.middleware.MorseMiddlewareClass):
    """ Handle communication between Blender and YARP."""

    def __init__(self, obj, parent=None):
        """ Initialize the network and connect to the yarp server."""
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self._yarpPorts = dict()
        self._component_ports = dict()

        # Get the Network attribute of yarp,
        #  then call its init method
        # Strange that we should do all this,
        #  probably because this file has the same name.
        # So this is the only way it seems to work
        self._yarp_module = sys.modules['yarp']
        #self._yarp_module.Network.init()
        self.yarp_object = self._yarp_module.Network()
        #yarp.Network.init()


    def __del__(self):
        """ Close all open YARP ports. """
        self.finalize()


    def register_component(self, component_name, component_instance, mw_data):
        """ Open the port used to communicate the specified component.

        The name of the port is postfixed with in/out, according to
        the direction of the communication. """
        # Compose the name of the port
        parent_name = component_instance.robot_parent.blender_obj.name
        port_name = 'robots/{0}/{1}'.format(parent_name, component_name)

        # Extract the information for this middleware
        # This will be tailored for each middleware according to its needs
        function_name = mw_data[1]
        remote = None
        if len(mw_data) > 2:
            remote = mw_data[2]

        function = self._check_function_exists(function_name)
        # The function exists within this class,
        #  so it can be directly assigned to the instance
        if function != None:
            # Data read functions
            if function_name == "read_message":
                port_name = port_name + '/in'
                self.registerBufferedPortBottle([port_name])
                component_instance.input_functions.append(function)
                #if remote != None:
                    #self.connectPorts(remote, "/ors/"+port_name)
                # Store the name of the port
                self._component_ports[component_name] = port_name
            # Data write functions
            elif function_name == "post_message":
                port_name = port_name + '/out'
                self.registerBufferedPortBottle([port_name])
                component_instance.output_functions.append(function)
                #if remote != None:
                    #self.connectPorts("/ors/"+port_name, remote)
                # Store the name of the port
                self._component_ports[component_name] = port_name
            # Image write functions
            elif function_name == "post_image_RGBA":
                port_name = port_name + '/out'
                self.registerBufferedPortImageRgba([port_name])
                component_instance.output_functions.append(function)
                #if remote != None:
                    #self.connectPorts("/ors/"+port_name, remote)
                # Store the name of the port
                self._component_ports[component_name] = port_name
            # If it is an external function that has already been added
            else:
                # Get the reference to the external module
                # (should be already included)
                source_file = mw_data[2]
                module_name = re.sub('/', '.', source_file)
                module = sys.modules[module_name]
                try:
                    # Call the init method of the new serialisation
                    # Sends the name of the function as a means to identify
                    #  what kind of port it should use.
                    module.init_extra_module(self, component_instance, function, mw_data)
                except AttributeError as detail:
                    logger.error("%s in module '%s'" % (detail, source_file))

        else:
            # If there is no such function in this module,
            #  try importing from another one
            try:
                # Insert the method in this class
                function = self._add_method(mw_data, component_instance)

            except IndexError as detail:
                logger.error("Method '%s' is not known, and no external module has been specified. Check the 'component_config.py' file for typos" % function_name)
                return


    def read_message(self, component_instance):
        """ Read incoming data from a simple port.

        The argument is a copy of the component instance.
        Data is writen directly into the 'local_data' dictionary
        of the component instance.
        Return 'True' is information could be read from the port, and
        'False' is there was any problem with the communication.
        """
        port_name = self._component_ports[component_instance.blender_obj.name]

        try:
            yarp_port = self.getPort(port_name)
        except KeyError as detail:
            logger.error("Specified port does not exist: ", detail)
            return False

        message_data = yarp_port.read(False)

        if message_data != None:
            # Data elements are of type defined in data_types
            i = 0
            for variable, data in component_instance.local_data.items():
                if isinstance(data, int):
                    msg_data = message_data.get(i).asInt()
                    component_instance.local_data[variable] = msg_data
                elif isinstance(data, float):
                    msg_data = message_data.get(i).asDouble()
                    component_instance.local_data[variable] = msg_data
                elif isinstance(data, str):
                    msg_data = message_data.get(i).toString()
                    component_instance.local_data[variable] = msg_data
                else:
                    logger.error("Unknown data type at 'read_message'")
                    logger.info("DATA: ", data, " | TYPE: ", type(data))
                i = i + 1
            return True

        else:
            return False



    def post_message(self, component_instance):
        """ Send the data of a component through a simple port.

        The argument is a copy of the component instance.
        """
        port_name = self._component_ports[component_instance.blender_obj.name]

        try:
            yarp_port = self.getPort(port_name)
        except KeyError as detail:
            logger.error("Specified port does not exist: ", detail)
            return

        bottle = yarp_port.prepare()
        bottle.clear()
        # Sort the data accodring to its type
        for variable, data in component_instance.local_data.items():
            if isinstance(data, int):
                bottle.addInt(data)
            elif isinstance(data, float):
                bottle.addDouble(data)
            elif isinstance(data, str):
                bottle.addString(data)
            else:
                logger.error("Unknown data type at 'post_message'")
                # Trying to store 'type(data)' causes an error in the logger.
                # Because the type is not directly printable.
                # This is caused when sending a dictionary type
                #logger.info("DATA: ", data, " | TYPE: ", type(data))

        #yarp_port.writeStrict()
        yarp_port.write()


    def post_image_RGBA(self, component_instance):
        """ Send an RGBA image through the given named port."""
        port_name = self._component_ports[component_instance.blender_obj.name]

        try:
            yarp_port = self.getPort(port_name)
        except KeyError as detail:
            logger.error("Specified port does not exist: ", detail)
            return

        # Wrap the data in a YARP image
        img = self._yarp_module.ImageRgba()
        img.setTopIsLowIndex(0)
        img.setQuantum(1)

        # Get the image data from the camera instance
        img_string = component_instance.local_data['image']
        img_X = component_instance.image_width
        img_Y = component_instance.image_height

        # Check that an image exists:
        if img_string != None and img_string != '':
            try:
                data = img_string
                # Pass the data as is, from the VideoTexture module
                # NOTE: This requires the patch to yarp-python bindings
                img.setExternal(data,img_X,img_Y)
            except TypeError as detail:
                logger.info("No image yet: %s" % detail)

            # Copy to image with "regular" YARP pixel order
            # Otherwise the image is upside-down
            img2 = yarp_port.prepare()
            img2.copy(img)

            # Write the image
            yarp_port.write()


    def registerBufferedPortBottle(self, portList):
        """ Create a new Buffered Port Bottle, given an identifying name. """
        for portName in portList:
            portName = '/ors/'+portName
            if portName not in self._yarpPorts:
                logger.debug('Yarp Mid: Adding ' + portName + ' buffered bottle port.')
                port = self._yarp_module.BufferedPortBottle()
                port.open(portName)
                self._yarpPorts[portName] = port
            else: raise NameError(portName + " port name already exist!")


    ## XXX You need a special patch to export Mono Interface througth swig
    def registerBufferedPortImageMono(self, portList):
        """ Create a new Buffered Port Bottle, given an identifying name.
            This is exclusively used for image data."""
        for portName in portList:
            portName = '/ors/'+portName
            if portName not in self._yarpPorts:
                logger.debug('Yarp Mid: Adding ' + portName + ' buffered image port.')
                port = self._yarp_module.BufferedPortImageMono()
                port.open(portName)
                self._yarpPorts[portName] = port
            else: raise NameError(portName + " port name already exist!")


    def registerBufferedPortImageRgb(self, portList):
        """ Create a new Buffered Port Bottle, given an identifying name.
            This is exclusively used for image data."""
        for portName in portList:
            portName = '/ors/'+portName
            if portName not in self._yarpPorts:
                logger.debug('Yarp Mid: Adding ' + portName + ' buffered image port.')
                port = self._yarp_module.BufferedPortImageRgb()
                port.open(portName)
                self._yarpPorts[portName] = port
            else: raise NameError(portName + " port name already exist!")


    def registerBufferedPortImageRgba(self, portList):
        """ Create a new Buffered Port Bottle, given an identifying name.
            This is exclusively used for image data."""
        for portName in portList:
            portName = '/ors/'+portName
            if portName not in self._yarpPorts:
                logger.debug('Yarp Mid: Adding ' + portName + ' buffered image port.')
                port = self._yarp_module.BufferedPortImageRgba()
                port.open(portName)
                self._yarpPorts[portName] = port
            else: raise NameError(portName + " port name already exist!")


    def registerPort(self, portList):
        """ Open a simple yarp port.
            Used to send image data (Works better than a buffered port)."""
        for portName in portList:
            portName = '/ors/'+portName
            if portName not in self._yarpPorts:
                logger.debug('Yarp Mid: Adding ' + portName + ' port.')
                port = self._yarp_module.Port()
                port.open(portName)
                self._yarpPorts[portName] = port
            else: raise NameError(portName + " port name already exist!")


    def finalize(self):
        """ Close all currently opened ports and release the network."""
        for port in self._yarpPorts.values():
            port.close()

        #self._network.fini()
        #self._yarp_module.Network.fini()
        #yarp.Network.fini()
        logger.info('Yarp Mid: ports have been closed.')


    def getPort(self, portName):
        """ Retrieve a yarp port associated to the given name."""
        port = '/ors/' + portName
        return self._yarpPorts[port]


    def printOpenPorts(self):
        """ Return a list of all currently opened ports."""
        logger.info("Yarp Mid: List of ports:")
        for name, port in self._yarpPorts.items():
            logger.info(" - Port name '{0}' = '{1}'".format(name, port))
            
    def connectPorts(self, source, target):
        """ Connect a yarp source to a specific yarp target.
            Can be used to publish or subscribe to a yarp topic. """
        self.yarp_object.connect(source, target)
