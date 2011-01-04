from morse.middleware.pocolibs.sensors.Gyro_Poster import ors_pom_poster

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
        poster_name = 'pom_{0}_{1}_Poster'.format(parent_name, component_name)

    poster_id = init_pom_poster(self, component_instance, poster_name)
    if poster_id != None:
        print ("Pocolibs created poster '%s' of type pom" % poster_id)
        component_instance.output_functions.append(function)
        # Store the name of the port
        self._poster_dict[component_name] = poster_id


def init_pom_poster(self, component_instance, poster_name):
    """ Prepare the data for a pom poster """

    reference_frame = component_instance.blender_obj['reference_frame']
    confidence = component_instance.blender_obj['confidence']
    poster_id, ok = ors_pom_poster.init_data(poster_name, reference_frame, confidence)
    if ok == 0:
        print ("ERROR creating poster. This module may not work")
        return None

    print ("pom poster ID: {0}".format(poster_id))
    return poster_id


def write_pom(self, component_instance):
    """ Write the sensor position to a poster

    The argument must be the instance to a morse gyroscope class. """
    # Get the id of the poster already created
    try:
        poster_id = self._poster_dict[component_instance.blender_obj.name]
    except AttributeError as detail:
        print ("Something BAD happened at POM poster: %s" % detail)
        sys.exit()

    main_to_origin = component_instance.robot_parent.position_3d

    pom_robot_position =  ors_pom_poster.pom_position()
    pom_robot_position.x = main_to_origin.x
    pom_robot_position.y = main_to_origin.y
    pom_robot_position.z = main_to_origin.z
    pom_robot_position.yaw = main_to_origin.yaw
    pom_robot_position.pitch = main_to_origin.pitch
    pom_robot_position.roll = main_to_origin.roll

    #print ("POM main_to_origin: %.4f, %.4f, %.4f" % (main_to_origin.x, main_to_origin.y, main_to_origin.z))

    main_to_sensor = component_instance.sensor_to_robot_position_3d()

    pom_sensor_position = ors_pom_poster.pom_position()
    pom_sensor_position.x = main_to_sensor.x
    pom_sensor_position.y = main_to_sensor.y
    pom_sensor_position.z = main_to_sensor.z
    pom_sensor_position.yaw = main_to_sensor.yaw
    pom_sensor_position.pitch = main_to_sensor.pitch
    pom_sensor_position.roll = main_to_sensor.roll

    #print ("POM main_to_sensor: %.4f, %.4f, %.4f" % (main_to_sensor.x, main_to_sensor.y, main_to_sensor.z))

    ors_pom_poster.write_pom_poster(poster_id, pom_robot_position, pom_sensor_position)

    """
    ors_pom_poster.post_data(poster_id,
            position3d.x, position3d.y,
            position3d.z, position3d.yaw,
            position3d.pitch, position3d.roll)
    """
