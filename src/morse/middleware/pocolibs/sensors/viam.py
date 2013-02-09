import logging; logger = logging.getLogger("morse." + __name__)
import sys
import math
from morse.core import blenderapi
from morse.middleware import AbstractDatastream
from morse.middleware.pocolibs.viam import *
from morse.middleware.pocolibs_datastream import poster_name, Pocolibs


class ViamPoster(AbstractDatastream):
    def initialize(self):
        name = poster_name(self.component_name, self.kwargs)
        self.camera_order = []

        """ Prepare the data for a viam poster """
        cameras = []
        pos_cam = []
        # Get the names of the data for the cameras
        for camera_name in self.component_instance.camera_list:
            camera_instance = blenderapi.persistantstorage().componentDict[camera_name]

            # Create an image structure for each camera
            image_init = { 'name': camera_name,
                           'width': int(camera_instance.image_width),
                           'height': int(camera_instance.image_height),
                           'focal': float(camera_instance.image_focal)
                         }

            pos_cam.append(camera_instance.bge_object.localPosition)
            cameras.append(image_init)
            self.camera_order.append(camera_name)

        baseline = 0
        # This is the case for a stereo camera
        if self.component_instance.num_cameras == 2:
            baseline = math.sqrt( math.pow(pos_cam[0][0] - pos_cam[1][0], 2) +
                                  math.pow(pos_cam[0][1] - pos_cam[1][1], 2) +
                                  math.pow(pos_cam[0][2] - pos_cam[1][2], 2))

            # viam expects first the left camera, then the right camera
            # Check the y difference between the two cameras
            if (pos_cam[0][1] < pos_cam[1][1]):
                cameras.reverse()
                self.camera_order.reverse()

            # Create the actual poster
            self.obj = Viam(name, 'stereo_bank', baseline, cameras)
        else:
            logger.info("The PTU sensor does not have two cameras attached. It is being disabled!")

    def default(self, ci):

        first_cam = blenderapi.persistantstorage().componentDict[self.camera_order[0]]
        main_to_origin = first_cam.robot_pose

        pom_robot_position =  ViamPos()
        pom_robot_position.x = main_to_origin.x
        pom_robot_position.y = main_to_origin.y
        pom_robot_position.z = main_to_origin.z
        pom_robot_position.yaw = main_to_origin.yaw
        pom_robot_position.pitch = main_to_origin.pitch
        pom_robot_position.roll = main_to_origin.roll

        # Compute the current time
        pom_date, t = Pocolibs.compute_date()

        ors_cameras = []
        ors_images = []

        # Cycle throught the cameras on the base
        # In normal circumstances, there will be two for stereo
        for camera_name in self.camera_order:
            camera_instance = blenderapi.persistantstorage().componentDict[camera_name]
            main_to_sensor = main_to_origin.transformation3d_with(
                    camera_instance.position_3d)
            imX = camera_instance.image_width
            imY = camera_instance.image_height
            try:
                image_data = camera_instance.local_data['image']
            except KeyError as detail:
                logger.warning("Camera image not found to read by VIAM poster.\n \
                        \tThe 'Class' property in the Camera component could be \
                        wrongly defined")

            # Don't create a poster if the camera is disabled
            if image_data == None or not camera_instance.capturing:
                logger.debug("Camera '%s' not capturing. Exiting viam poster" % \
                        camera_instance.bge_object.name)
                return

            # Fill in the structure with the image information
            camera_data = ViamSimuImage()
            camera_data.width = imX
            camera_data.height = imY
            camera_data.pom_tag = pom_date
            camera_data.tacq_sec = t.second
            camera_data.tacq_usec = t.microsecond
            camera_data.x = main_to_sensor.x
            camera_data.y = main_to_sensor.y
            camera_data.z = main_to_sensor.z
            camera_data.yaw = main_to_sensor.yaw 
            camera_data.pitch = main_to_sensor.pitch + math.pi # XXX WTF 
            camera_data.roll = main_to_sensor.roll
            camera_data.flipped = camera_instance.vertical_flip

            ors_cameras.append(camera_data)
            ors_images.append(image_data)

        self.obj.post(pom_robot_position, ors_cameras, ors_images)

    def finalize(self):
        self.obj.finalize()
        AbstractDatastream.finalize(self)
