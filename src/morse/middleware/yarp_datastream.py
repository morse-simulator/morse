import logging; logger = logging.getLogger("morse." + __name__)
import sys
import re
import yarp
import mathutils
from morse.helpers.transformation import Transformation3d
from morse.middleware.abstract_datastream import AbstractDatastream
from morse.core.datastream import *

class YarpPort(AbstractDatastream):
    def initialize(self, port_creator_fn, is_input):
        self.is_input = is_input
        self.port_name = self._get_port_name()
        self.port = port_creator_fn()
        self.port.open(self.port_name)
        if 'topic' in self.kwargs:
            self.connect2topic(self.kwargs['topic'])

    def _get_port_name(self):
        if 'port' in self.kwargs:
            return self.kwargs['port']
        else:
            port_name = re.sub(r'\.([0-9]+)', r'\1', self.component_name)
            # '/robot001/sensor001'
            return '/morse/' + port_name.replace('.', '/') + \
                   ('/in' if self.is_input else '/out')

    def connect2topic(self, topic):
        """ Connect a yarp port to a specific yarp topic. """
        if self.is_input:
            yarp.Network.connect("topic://"+topic, self.port)
        else:
            yarp.Network.connect(self.port, "topic://"+topic)

    def finalize(self):
        self.port.close()

class YarpPublisher(YarpPort):
    _type_name = "yarp::Bottle"

    def initialize(self):
        YarpPort.initialize(self, yarp.BufferedPortBottle, False)

    def encode_message(self, bottle, data, component_name):
        """ Prepare the content of the bottle 

        This function can be recursively called in case of list processing
        """
        if isinstance(data, int):
            bottle.addInt(data)
        elif isinstance(data, float):
            bottle.addDouble(data)
        elif isinstance(data, str):
            bottle.addString(data)
        elif isinstance(data, list):
            m_bottle = bottle.addList()
            for m_data in data:
                self.encode_message(m_bottle, m_data, component_name)
        elif isinstance(data, dict):
            for key, value in data.items():
                m_bottle = bottle.addList()
                self.encode_message(m_bottle, key, component_name)
                self.encode_message(m_bottle, value, component_name)


        elif isinstance(data, mathutils.Vector):
            self.encode_message(m_bottle, data[:], component_name)
        elif isinstance(data, mathutils.Matrix):
            self.encode_message(m_bottle, data[:], component_name)
        elif isinstance(data, mathutils.Quaternion):
            self.encode_message(m_bottle,
                    [data.x , data.y, data.z, data.w], component_name)
        elif isinstance(data, mathutils.Euler):
            self.encode_message(m_bottle,
                    [data.z , data.y, data.x], component_name)
        elif isinstance(data, Transformation3d):
            self.encode_message(m_bottle,
                    [data.x , data.y, data.z,
                     data.yaw, data.pitch, data.roll], component_name)
        else:
            logger.error("Unknown data type in component '%s'" % component_name)

    def default(self, ci):
        bottle = self.port.prepare()
        bottle.clear()
        self.encode(bottle)
        self.port.write()

    def encode(self, bottle):
        for data in self.data.values():
            self.encode_message(bottle, data, self.component_name)

class YarpImagePublisher(YarpPort):

    _type_name = "yarp::ImageRGBA"

    def initialize(self):
        YarpPort.initialize(self, yarp.BufferedPortImageRgba, False)

    def default(self, ci):
        # Wrap the data in a YARP image
        img = yarp.ImageRgba()
        img.setTopIsLowIndex(0)
        img.setQuantum(1)

        # Get the image data from the camera instance
        img_string = self.data['image']
        img_X = self.component_instance.image_width
        img_Y = self.component_instance.image_height

        # Check that an image exists:
        if img_string != None and img_string != '':
            try:
                data = img_string
                # Pass the data as is, from the bge.texture module
                # NOTE: This requires the patch to yarp-python bindings
                img.setExternal(data,img_X,img_Y)
            except TypeError as detail:
                logger.info("No image yet: %s" % detail)

            # Copy to image with "regular" YARP pixel order
            # Otherwise the image is upside-down
            img2 = self.port.prepare()
            img2.copy(img)

            # Write the image
            self.port.write()

class YarpReader(YarpPort):
    _type_name = "yarp::Bottle"

    def initialize(self):
        YarpPort.initialize(self, yarp.BufferedPortBottle, True)

    def default(self, ci):
        message_data = self.port.read(False)

        if message_data != None:
            # Data elements are of type defined in data_types
            i = 0
            for variable, data in self.data.items():
                if isinstance(data, int):
                    msg_data = message_data.get(i).asInt()
                    self.data[variable] = msg_data
                    logger.debug("read %s as %d" % (variable, msg_data))
                elif isinstance(data, float):
                    msg_data = message_data.get(i).asDouble()
                    self.data[variable] = msg_data
                    logger.debug("read %s as %f" % (variable, msg_data))
                elif isinstance(data, str):
                    msg_data = message_data.get(i).toString()
                    self.data[variable] = msg_data
                    logger.debug("read %s as %s" % (variable, msg_data))
                else:
                    logger.error("Unknown data type at 'read_message', "
                                 "with component '%s'" % self.component_name)
                    logger.info("DATA: ", data, " | TYPE: ", type(data))
                i = i + 1
            return True

        else:
            return False

class Yarp(Datastream):
    """ Handle communication between Blender and YARP."""

    def __init__(self):
        """ Initialize the network and connect to the yarp server."""
        # Call the constructor of the parent class
        super(self.__class__,self).__init__()

        yarp.Network.init()


    def __del__(self):
        """ Close all open YARP ports. """
        self.finalize()

    def finalize(self):
        """ Close all currently opened ports and release the network."""
        yarp.Network.fini()
        logger.info('Yarp Mid: ports have been closed.')


