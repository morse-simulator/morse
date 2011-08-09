import logging; logger = logging.getLogger("morse." + __name__)
import sys
import math
import GameLogic
from morse.middleware.pocolibs.sensors.Viam_Poster import ors_viam_poster

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    component_name = component_instance.blender_obj.name
    parent_name = component_instance.robot_parent.blender_obj.name
    # Check if the name of the poster has been given in mw_data
    try:
        # It should be the 4th parameter
        poster_name = mw_data[3]
    except IndexError:
        # Compose the name of the poster, based on the parent and module names
        poster_name = 'viam_{0}_{1}_Poster'.format(parent_name, component_name)

    poster_id = init_viam_poster(component_instance, poster_name)
    if poster_id != None:
        logger.info("Pocolibs created poster '%s' of type viam" % poster_id)
        component_instance.output_functions.append(function)
        # Store the name of the port
        self._poster_dict[component_name] = poster_id


def init_viam_poster(component_instance, poster_name):
    """ Prepare the data for a viam poster """
    cameras = []
    pos_cam = []
    # Get the names of the data for the cameras
    for camera_name in component_instance.camera_list:
        camera_instance = GameLogic.componentDict[camera_name]

        # Create an image structure for each camera
        image_init = ors_viam_poster.simu_image_init()
        image_init.camera_name = camera_name
        image_init.width = camera_instance.image_width
        image_init.height = camera_instance.image_height
        image_init.focal = camera_instance.image_focal
        pos_cam.append(camera_instance.blender_obj.position)
        cameras.append(image_init)

    baseline = 0
    # This is the case for a stereo camera
    if component_instance.num_cameras == 2:
        baseline = math.sqrt( math.pow(pos_cam[0][0] - pos_cam[1][0], 2) +
                              math.pow(pos_cam[0][1] - pos_cam[1][1], 2) +
                              math.pow(pos_cam[0][2] - pos_cam[1][2], 2))

        # Create the actual poster
        poster_id, ok = ors_viam_poster.init_data(poster_name, "stereo_bank", \
                                            component_instance.num_cameras, \
                                            baseline, cameras[0], cameras[1])
        if ok == 1:
            logger.info("viam poster ID: {0}".format(poster_id))
            return poster_id

        else:
        #elif ok == 0:
            logger.error("Creating poster. This module may not work")
            return None
    # What to do if there is no second camera???
    #elif component_instance.num_cameras == 1:
    else:
        logger.info("The PTU sensor does not have two cameras attached. It is being disabled!")
        return None



def write_viam(self, component_instance):
    """ Write an image and all its data to a poster """
    # Get the id of the poster already created
    try:
        poster_id = self._poster_dict[component_instance.blender_obj.name]
    except AttributeError as detail:
        logger.error("Something BAD happened at viam poster: %s" % detail)
        sys.exit()
    parent = component_instance.robot_parent

    main_to_origin = parent.position_3d

    pom_robot_position =  ors_viam_poster.pom_position()
    pom_robot_position.x = main_to_origin.x
    pom_robot_position.y = main_to_origin.y
    pom_robot_position.z = main_to_origin.z
    pom_robot_position.yaw = main_to_origin.yaw
    pom_robot_position.pitch = main_to_origin.pitch
    pom_robot_position.roll = main_to_origin.roll

    # Compute the current time
    pom_date, t = self._compute_date()

    ors_cameras = []
    ors_images = []

    # Cycle throught the cameras on the base
    # In normal circumstances, there will be two for stereo
    for camera_name in component_instance.camera_list:
        camera_instance = GameLogic.componentDict[camera_name]

        main_to_sensor = camera_instance.sensor_to_robot_position_3d()

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
                    camera_instance.blender_obj.name)
            return

        # Fill in the structure with the image information
        camera_data = ors_viam_poster.simu_image()
        camera_data.width = imX
        camera_data.height = imY
        camera_data.pom_tag = pom_date
        camera_data.tacq_sec = t.second
        camera_data.tacq_usec = t.microsecond
        camera_data.sensor = ors_viam_poster.pom_position()
        camera_data.sensor.x = main_to_sensor.x
        camera_data.sensor.y = main_to_sensor.y
        camera_data.sensor.z = main_to_sensor.z
        camera_data.sensor.yaw = main_to_sensor.yaw 
        camera_data.sensor.pitch = main_to_sensor.pitch
        camera_data.sensor.roll = main_to_sensor.roll

        ors_cameras.append(camera_data)
        ors_images.append(image_data)

    # Write to the poster with the data for both images
    ors_viam_poster.post_viam_poster(poster_id, pom_robot_position, \
                                     component_instance.num_cameras, \
                                     ors_cameras[0], ors_images[0], \
                                     ors_cameras[1], ors_images[1])
