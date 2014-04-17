import logging; logger = logging.getLogger("morse." + __name__)
from morse.core import blenderapi
from morse.core.sensor import Sensor
from morse.helpers.components import add_data, add_property, add_level
from morse.builder import bpymorse
"""
Important note:

    The 'logger.debug' instructions take some processor work, even if they are
    not displayed. For this reason, it is best to comment out these lines in
    the 'default_action' method.
"""

class LaserScanner(Sensor):
    """
    This is a generic sensor class used to emulate laser range scanners,
    including a variety of SICK and Hokuyo sensors.

    This sensor works by generating a series of rays in predefined directions, and
    then computing whether any active object is found within a certain distance
    from the origin of the sensor.

    The resolution and detection range can be completely configured using the MORSE
    Builder API.  This will generate a flat mesh with a semi-circular shape, where
    its vertices represent the directions in which the rays of the sensor are cast.
    It is also possible to create a sensor with multiple scan layers, such as the
    SICK LD-MRS. This is configured using the parameters specified below.

    .. note::

        Objects in the scene with the **No collision** setting in their Game
        properties will not be detected by this sensor


    +-----------------------------------------------------------+------------------------------------------------------------------+
    | .. figure:: ../../../media/sensors/laserscanners/sick.png | .. figure:: ../../../media/sensors/laserscanners/sick-ld-mrs.png |
    |    :width: 200                                            |    :width: 200                                                   |
    |                                                           |                                                                  |
    |    SICK LMS500                                            |    SICK LD-MRS                                                   |
    +-----------------------------------------------------------+------------------------------------------------------------------+
    |.. figure:: ../../../media/sensors/laserscanners/hokuyo.png|                                                                  |
    |   :width: 200                                             |                                                                  |
    |                                                           |                                                                  |
    |   Hokuyo                                                  |                                                                  |
    +-----------------------------------------------------------+------------------------------------------------------------------+

    LaserScanner with remission values
    ___________________________________

    Remission "is the reflection or scattering of light by a material." 
    (http://en.wikipedia.org/wiki/Remission_%28spectroscopy%29)

    The level "rssi" adds a list of remission values to the LaserScanner. If a ray 
    during the scan hits an object the rssi-value is the specular intenisty of the 
    object's material; If it does not hit an object with a material the remission 
    value is set to 0. 

    The intensity of the material can be changed in Blender (Property -> Material -> 
    Specular -> Intensity). The important options are highlighted in the first image.

    +---------------------------------------------------------------------------------+
    | .. figure:: ../../../media/rssi_blender_intensity_material.png                  |
    |    :width: 300                                                                  |
    |                                                                                 |
    |    Specular intensity of a material in Blender                                  |
    +---------------------------------------------------------------------------------+
    | .. figure:: ../../../media/rssi_laserscanner_example.png                        |
    |    :width: 300                                                                  |
    |                                                                                 |
    |    Example of the LaserScanner with remission values                            |
    +---------------------------------------------------------------------------------+

    In the second image the sensor is illustrated. Above every box the material 
    properties and a corresponding excerpt from the socket stream is displayed.

    .. note::
        
        The remission values are **not** comparable to any physical remission value 
        and are **not** calculated. They are just based on a property of a visual effect.
    

    Configuration of the scanning parameters
    ----------------------------------------

    The number and direction of the rays emitted by the sensor is determined by the
    vertices of a semi-circle mesh parented to the sensor. The sensor will cast
    rays from the center of the sensor in the direction of each of the vertices in
    the semi-circle.

    Three preconfigured scanners are available: a **SICK LMS500** laser scanner, a
    **Hokuyo** and a **SICK LD-MRS**. The example below shows how to add them
    in a simulation:

    .. code-block:: python

        from morse.builder import *

        # Append a sick laser
        sick = Sick() # range: 30m, field: 180deg, 180 sample points

        hokuyo = Hokuyo() # range: 30m, field: 270deg, 1080 sample points

        sick_ld_mrs = SickLDMRS() # range: 30m, field 100deg, 4 layers, 400 points per layer

    All these default parameters can be changed i(cf *Configuration
    parameters* below). An example of how to change the arc object using the
    Builder API is show below:

    .. code-block:: python

        from morse.builder import *

        # Append a sick laser
        sick = Sick()
        sick.properties(resolution = 5)
        sick.properties(scan_window = 90)
        sick.properties(laser_range = 5.0)

    .. note::

        In some special cases (like multi-robot setups), you may need to
        additionally call ``sick.create_sick_arc()`` after setting your
        scanner properties.

        The ray will be created from (-window/2) to (+window/2). So the
        ``range_list`` will contain the range clockwise.


    Another example for the SICK LD-MRS:

    .. code-block:: python

        from morse.builder import *

        sick = SickLDMRS()
        sick.properties(Visible_arc = True)
        sick.properties(resolution = 1.0)
        sick.properties(scan_window = 100)
        sick.properties(laser_range = 50.0)
        sick.properties(layers = 4)
        sick.properties(layer_separation = 0.8)
        sick.properties(layer_offset = 0.25)

    As with any other component, it is possible to adjust the refresh frequency of
    the sensor, after it has been defined in the builder script. For example, to
    set the frequency to 1 Hz:

    .. code-block:: python

        sick.frequency(1.0)
    """

    _name = "Laser Scanner Sensors"
    _short_desc = "Generic laser range sensors"

    add_level("raw", None, doc = "raw laserscanner: \
                    Laserscan with point_list and range_list", default = True )
    add_level("rssi", "morse.sensors.laserscanner.RSSILaserScanner", doc = "laserscanner with rssi: \
                    Laserscan with point_list, range_list and remission_list")

    add_data('point_list', [], "list", "Array that stores the positions of \
            the points found by the laser. The points are given with respect \
            to the location of the sensor, and stored as lists of three \
            elements. The number of points depends on the geometry of the arc \
            parented to the sensor (see below). The point (0, 0, 0) means that\
            this ray has not it anything in its range", level =["raw", "rssi"] )
    add_data('range_list', [], "list", "Array that stores the distance to the \
            first obstacle detected by each ray. The order indexing of this \
            array is the same as for point_list, so that the element in the \
            same index of both lists will correspond to the measures for the \
            same ray. If the ray does not hit anything in its range it returns \
            laser_range", level =["raw", "rssi"])
    add_data('remission_list', [], "list", "Array that stores the remission \
            value for the points found by the laser. The specular intensity \
            is set as the remission value. If no object is hit, the remission \
            value is set to 0", level ="rssi")

    add_property('laser_range', 30.0, 'laser_range', "float",
                 "The distance in meters from the center of the sensor to which\
                  it is capable of detecting other objects.")
    add_property('resolution', 1.0, 'resolution', "float",
                 "The angle between each laser in the sensor. Expressed in \
                  degrees in decimal format. (i. e.), half a degree is     \
                  expressed as 0.5. Used when creating the arc object.")
    add_property('scan_window', 180.0, 'scan_window', "float",
                 "The full angle covered by the sensor. Expressed in degrees \
                  in decimal format. Used when creating the arc object.")
    add_property('visible_arc', False, 'Visible_arc', "boolean",
                 "if the laser arc should be displayed during the simulation")
    add_property('layers', 1, 'layers', "integer",
                  "Number of scanning planes used by the sensor.")
    add_property('layer_separation', 0.8, 'layer_separation', "float",
                 "The angular distance between the planes, in degrees.")
    add_property('layer_offset', 0.125, 'layer_offset', "float",
                 "The horizontal distance between the scan points in \
                  consecutive scanning layers. Must be given in degrees.")

    def __init__(self, obj, parent=None):
        """
        Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        Sensor.__init__(self, obj, parent)

        arc_prefix = 'Arc_'

        # Look for a child arc to use for the scans
        for child in obj.children:
            if arc_prefix in child.name:
                self._ray_arc = child
                logger.info("Sick: Using arc object: '%s'" % self._ray_arc)
                break

        # Set its visibility, according to the settings
        self._ray_arc.setVisible(self.visible_arc)
        self._ray_list = []

        # Create an empty list to store the intersection points
        self.local_data['point_list'] = []
        self.local_data['range_list'] = []


        # Get the datablock of the arc, to extract its vertices
        ray_object = blenderapi.objectdata(self._ray_arc.name)
        for vertex in ray_object.data.vertices:
            logger.debug ("Vertex %d = %s" % (vertex.index, vertex.co))

            # Skip the first vertex.
            # It is the one located at the center of the sensor
            if vertex.index == 0:
                continue

            # Store the position of the vertex in a list
            # The position is already given as a mathutils.Vector
            self._ray_list.append(vertex.co)

            # Insert empty points into the data list
            self.local_data['point_list'].append([0.0, 0.0, 0.0])
            # Insert zeros into the range list
            self.local_data['range_list'].append(0.0)

            logger.debug("RAY %d = [%.4f, %.4f, %.4f]" %
                         (vertex.index, self._ray_list[vertex.index-1][0],
                                        self._ray_list[vertex.index-1][1],
                                        self._ray_list[vertex.index-1][2]))

        # Get some information to be able to deform the arcs
        if self.visible_arc:
            self._layers = 1
            if 'layers' in self.bge_object:
                self._layers = self.bge_object['layers']
            self._vertex_per_layer = len(self._ray_list) // self._layers

        logger.info('Component initialized, runs at %.2f Hz', self.frequency)


    def default_action(self):
        """
        Do ray tracing from the SICK object using a semicircle

        Generates a list of lists, with the points located.
        Also deforms the geometry of the arc associated to the SICK,
        as a way to display the results obtained.
        """
        #logger.debug("ARC POSITION: [%.4f, %.4f, %.4f]" %
        #                (self.bge_object.position[0],
        #                 self.bge_object.position[1],
        #                 self.bge_object.position[2]))

        # Get the inverse of the transformation matrix
        inverse = self.position_3d.matrix.inverted()

        index = 0
        for ray in self._ray_list:
            # Transform the ray to the current position and rotation
            #  of the sensor
            correct_ray = self.position_3d.matrix * ray

            # Shoot a ray towards the target
            target, point, normal = self.bge_object.rayCast(correct_ray, None,
                                                             self.laser_range)

            #logger.debug("\tTarget, point, normal: %s, %s, %s" %
            #               (target, point, normal))

            # Register when an intersection occurred
            if target:
                distance = self.bge_object.getDistanceTo(point)
                # Return the point to the reference of the sensor
                new_point = inverse * point

                #logger.debug("\t\tGOT INTERSECTION WITH RAY: [%.4f, %.4f, %.4f]" % (correct_ray[0], correct_ray[1], correct_ray[2]))
                #logger.debug("\t\tINTERSECTION AT: [%.4f, %.4f, %.4f] = %s" % (point[0], point[1], point[2], target))
            # If there was no intersection, store the default values
            else:
                distance = self.laser_range
                new_point = [0.0, 0.0, 0.0]

            # Save the information gathered
            self.local_data['point_list'][index] = new_point[:]
            self.local_data['range_list'][index] = distance
            index += 1
            self.change_arc()


    def change_arc(self):
        # Change the shape of the arc to show what the sensor detects
        # Display only for 1 layer scanner
        if (2, 65, 0) < blenderapi.version() <= (2, 66, 3):
            # see http://projects.blender.org/tracker/?func=detail&aid=34550
            return # not supported in 2.66 due to BGE bug #34550
        # TODO rework the LDMRS (3 layers) display [code in 1.0-beta2]
        if self.visible_arc and self._layers == 1:
            for mesh in self._ray_arc.meshes:
                for m_index in range(len(mesh.materials)):
                    # Skip the first vertex (located at the center of the sensor)
                    for v_index in range(1, mesh.getVertexArrayLength(m_index)):
                        vertex = mesh.getVertex(m_index, v_index)
                        point = self.local_data['point_list'][v_index-1]
                        if point == [0.0, 0.0, 0.0]:
                            # If there was no intersection, move the vertex
                            # to the laser range
                            point = self._ray_list[v_index-1] * self.laser_range
                        vertex.setXYZ(point)


class LaserScannerRotationZ(LaserScanner):
    """Used for Velodyne sensor"""
    def default_action(self):
        LaserScanner.default_action(self)
        self.applyRotationZ()
    def applyRotationZ(self, rz=.01745):
        # The second parameter specifies a "local" movement
        self.bge_object.applyRotation([0, 0, rz], True)

class RSSILaserScanner(LaserScanner):

    def __init__(self, obj, parent=None):
        LaserScanner.__init__(self, obj, parent)
        self.local_data['remission_list'] = []
        ray_object = blenderapi.objectdata(self._ray_arc.name)
        for vertex in ray_object.data.vertices:
            # Skip the first vertex.
            # It is the one located at the center of the sensor
            if vertex.index == 0:
                continue

            # Insert zeros in the remission list
            self.local_data['remission_list'].append(0.0) 
       
    def getRSSIValue(self, target):

        """
        IMPORTANT:
        To get the material property a workaround is needed.
        The material from the targets is KX_BlenderMaterial (and not
        KX_PolygonMaterial). So the properties are not accessible.
        Workaround:
        Get material name of the GameObject and then use the method
        bpymorse.get_material(name).
        Material name must be parsed because of the prefix 'MA' in Blender;
        before the name is used to get the material properties.
        """
        mat_name = target.getMaterialName()
        try: 
            mat_name = mat_name[2:]
            mat = bpymorse.get_material(mat_name)
            if mat:
                return mat.specular_intensity

        except:
            logger.error("Error: Could not parse material name %s. \
                    The leading 'MA' (prefix in Blender) is missing. \
                    Please check on the material name and the source file \
                    /src/morse/sensors/laserscanner.py the method \
                    'default_action' in the class LaserScannner_RSSI, \
                    where the name is parsed."%mat_name)
            return -1              

    def default_action(self):
        inverse = self.position_3d.matrix.inverted()

        index = 0
        for ray in self._ray_list:
            # Transform the ray to the current position and rotation
            #  of the sensor
            correct_ray = self.position_3d.matrix * ray

            # Shoot a ray towards the target
            """
            target, point, normal = self.bge_object.rayCast(correct_ray, None,
                                                             self.laser_range)
            would be possible, but longer way to get material name:
            target -> (list)meshes -> (list)materials -> (string)getMaterialName(id)
            
            target_poly is shorter
            """
            target, point, normal, target_poly = self.bge_object.rayCast(correct_ray, None,
                                                             self.laser_range, "", 1, 1, 1)

            #logger.debug("\tTarget, point, normal: %s, %s, %s" %
            #               (target, point, normal))

            # Register when an intersection occurred


            if target_poly:
                distance = self.bge_object.getDistanceTo(point)
                new_point = inverse * point

                rssi = self.getRSSIValue(target_poly)                        
                            
                # Return the point to the reference of the sensor


                #logger.debug("\t\tGOT INTERSECTION WITH RAY: [%.4f, %.4f, %.4f]" % (correct_ray[0], correct_ray[1], correct_ray[2]))
                #logger.debug("\t\tINTERSECTION AT: [%.4f, %.4f, %.4f] = %s" % (point[0], point[1], point[2], target))
            # If there was no intersection, store the default values
            else:
                distance = self.laser_range
                new_point = [0.0, 0.0, 0.0]
                rssi = 0

            # Save the information gathered
            self.local_data['point_list'][index] = new_point[:]
            self.local_data['range_list'][index] = distance
            self.local_data['remission_list'][index] = rssi
            index += 1
            LaserScanner.change_arc(self)

