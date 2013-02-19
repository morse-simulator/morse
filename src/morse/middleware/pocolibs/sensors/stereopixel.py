import logging; logger = logging.getLogger("morse." + __name__)
import sys
import math
from morse.core import blenderapi
from morse.middleware import AbstractDatastream
from morse.middleware.pocolibs.stereopixel import *
from morse.middleware.pocolibs_datastream import poster_name, Pocolibs


class Spix3DImagePoster(AbstractDatastream):
    _type_name = "Spix3DImage"
    _type_url = "http://trac.laas.fr/git/stereopixel-genom/tree/stereopixelClient.h#n57"

    def initialize(self):
        name = poster_name(self.component_name, self.kwargs)
        self.obj = Stereopixel(name, self.component_instance.image_width,
                                     self.component_instance.image_height)

    def default(self, ci):

        main_to_origin = self.component_instance.robot_parent.position_3d
        pom_date, t = Pocolibs.compute_date()
        main_to_sensor = main_to_origin.transformation3d_with(
                    self.component_instance.position_3d)

        info = StereopixelSimu()
        info.width = self.component_instance.image_width
        info.height = self.component_instance.image_height

        info.x_rob = main_to_origin.x
        info.y_rob = main_to_origin.y
        info.z_rob = main_to_origin.z
        info.yaw_rob = main_to_origin.yaw
        info.pitch_rob = main_to_origin.pitch
        info.roll_rob = main_to_origin.roll

        info.x_cam = main_to_sensor.x
        info.y_cam = main_to_sensor.y
        info.z_cam = main_to_sensor.z
        info.yaw_cam = main_to_sensor.yaw
        info.pitch_cam = main_to_sensor.pitch
        info.roll_cam = main_to_sensor.roll

        info.pom_tag = pom_date

        self.obj.post(info, self.data['points'])

    def finalize(self):
        self.obj.finalize()
        AbstractDatastream.finalize(self)
