import logging; logger = logging.getLogger("morse." + __name__)
from morse.core import blenderapi
from math import pi
from morse.middleware import AbstractDatastream
from morse.middleware.pocolibs_datastream import poster_name, PocolibsDatastreamManager

try:
    from morse.middleware.pocolibs.velodyne import *
except ImportError:
    if not blenderapi.fake:
        raise


class Velodyne3DImage(AbstractDatastream):
    _type_name = "velodyne3DImage"
    _type_url = "http://trac.laas.fr/git/velodyne-genom/tree/velodyneClient.h#n48"

    def initialize(self):
        name = poster_name(self.component_name, self.kwargs)
        num_rotation = int(2 * pi / self.component_instance.rotation) + 1
        logger.info("num_rotation %d" % num_rotation)
        self.obj = Velodyne(name, self.component_instance.image_width,
                                     self.component_instance.image_height, num_rotation)

    def default(self, ci):
        if not self.component_instance.capturing:
            return

        main_to_origin = self.component_instance.robot_parent.position_3d
        pom_date, t = PocolibsDatastreamManager.compute_date()
        main_to_sensor = main_to_origin.transformation3d_with(
                    self.component_instance.position_3d)

        info = VelodyneSimu()
        info.nb_pts = self.data['nb_points']

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
