import logging; logger = logging.getLogger("morse." + __name__)
import GameLogic
from morse.middleware.pocolibs.sensors.Velodyne_Poster import ors_velodyne_poster

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
    except IndexError as detail:
        # Compose the name of the poster, based on the parent and module names
        poster_name = 'velodyne_{0}_{1}'.format(parent_name, component_name)

    poster_id = init_velodyne_poster(self, component_instance, poster_name)
    if poster_id != None:
        logger.info("Pocolibs created poster '%s' of type velodyne" % poster_id)
        component_instance.output_functions.append(function)
        # Store the name of the port
        self._poster_dict[component_name] = poster_id


def init_velodyne_poster(self, component_instance, poster_name):
    """ Prepare the data for a velodyne poster """
    # Constants defined in velodyne/velodyneClient.h
    VELODYNE_NB_LINES = 750
    VELODYNE_NB_COLUMNS = 384

    # Arbitrary value for how many lines will be included in a full
    #  turn of the sensor. Will depend on the speed given in the
    #  Logic Bricks
    self._PACKETS_PER_TURN = 60

    # Measure the amount of coordinates that will be stored in a single array
    array_size = VELODYNE_NB_LINES * VELODYNE_NB_COLUMNS * 3
    #array_size = len(component_instance.local_data['point_list']) * 3
    logger.infor("AT 'init_velodyne_poster' ARRAY SIZE IS %d" % array_size)
    poster_id, ok = ors_velodyne_poster.init_data(poster_name)
    if ok == 0:
        logger.error("Creating poster. The Velodyne Pocolib export module may not work")
        return None

    logger.info("Velodyne Poster '%s' created (ID: %d)" % (poster_name, poster_id))

    # Create the array using the type mapped in ors_velodyne_poster.i
    self._coordinate_array = ors_velodyne_poster.doubleArray(array_size)
    for i in range(array_size):
        self._coordinate_array[i] = 0.0

    # Declare counters for the blocks and packets
    self._blocks = 0
    self._packets = 0
    self._array_index = 0

    return poster_id


def write_velodyne(self, component_instance):
    """ Write the velodyne data to a Pocolibs poster.
    
    This method simply passes the values of the component_instance.local_data 
    dictionary that contains the 3D points captured by the sensor.
    """
    # Fill in the structure with the image information
    for point in component_instance.local_data['point_list']:
        logger.debug("Storing point: [%.4f, %.4f, %.4f]" % (point[0], point[1], point[2]))
        for i in range(3):
            self._coordinate_array[self._array_index] = point[i]
        self._array_index = self._array_index + 1

    # Increment the counters for blocks and packets
    self._blocks = self._blocks + 1
    if self._blocks == 6:
        self._blocks = 0
        self._packets = self._packets + 1

    # Determine when enough points have been scanned to send the poster
    if self._packets >= self._PACKETS_PER_TURN:
        # Get the id of the poster already created
        try:
            poster_id = self._poster_dict[component_instance.blender_obj.name]
        except AttributeError as detail:
            logger.error("Something BAD happened at velodyne poster: %s" % detail)
            sys.exit()

        # Compute the current time
        pom_date, t = self._compute_date()

        parent = component_instance.robot_parent
        main_to_origin = parent.position_3d
        main_to_sensor = component_instance.sensor_to_robot_position_3d()

        pom_robot_position =  ors_velodyne_poster.pom_position()
        pom_robot_position.x = main_to_origin.x
        pom_robot_position.y = main_to_origin.y
        pom_robot_position.z = main_to_origin.z
        pom_robot_position.yaw = main_to_origin.yaw
        pom_robot_position.pitch = main_to_origin.pitch
        pom_robot_position.roll = main_to_origin.roll

        pom_sensor_position =  ors_velodyne_poster.pom_position()
        pom_sensor_position.x = main_to_sensor.x
        pom_sensor_position.y = main_to_sensor.y
        pom_sensor_position.z = main_to_sensor.z
        pom_sensor_position.yaw = main_to_sensor.yaw
        pom_sensor_position.pitch = main_to_sensor.pitch
        pom_sensor_position.roll = main_to_sensor.roll

        logger.info("Writing a poster with %d packets and %d points" % (self._packets, self._array_index))
        # Write to the poster with the data for both images
        ors_velodyne_poster.post_velodyne_poster(poster_id, pom_date, \
                pom_robot_position, pom_sensor_position, self._packets, \
                self._array_index, self._coordinate_array)

        # Reset the counters
        self._blocks = 0
        self._packets = 0
        self._array_index = 0
        # Clean the array
        #for i in range(self._array_index):
            #self._coordinate_array[i] = 0.0
