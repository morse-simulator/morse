import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.yarp_datastream import YarpPort
import yarp

class YarpImagePublisher(YarpPort):

    _type_name = "yarp::ImageRGBA"

    def initialize(self):
        YarpPort.initialize(self, yarp.BufferedPortImageRgba, False)

    def default(self, ci):
        # Wrap the data in a YARP image
        img = self.port.prepare()
        img.setQuantum(1)

        # Get the image data from the camera instance
        img_string = self.data['image']
        img_x = self.component_instance.image_width
        img_y = self.component_instance.image_height

        # Check that an image exists:
        if img_string is not None and img_string != '':
            try:
                data = img_string
                img.setExternal(data, img_x, img_y)
            except TypeError as detail:
                logger.info("No image yet: %s" % detail)

            # Write the image
            self.port.write()
