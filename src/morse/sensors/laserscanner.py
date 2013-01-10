import logging; logger = logging.getLogger("morse." + __name__)
from morse.core import blenderapi
import morse.core.sensor
from morse.helpers.components import add_data, add_property

"""
Important note:

    The 'logger.debug' instructions take some processor work, even if they are
    not displayed. For this reason, it is best to comment out these lines in
    the 'default_action' method.
"""

class LaserScannerClass(morse.core.sensor.Sensor):
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

    add_data('point_list', [], "list", "Array that stores the positions of \
            the points found by the laser. The points are given with respect \
            to the location of the sensor, and stored as lists of three \
            elements. The number of points depends on the geometry of the arc \
            parented to the sensor (see below).")
    add_data('range_list', [], "list", "Array that stores the distance to the \
            first obstacle detected by each ray. The order indexing of this \
            array is the same as for point_list, so that the element in the \
            same index of both lists will correspond to the measures for the \
            same ray.")

    add_property('laser_range', 30.0, 'laser_range', "float", "The distance in meters from the center of the sensor to which it is capable of detecting other objects.")
    add_property('resolution', 1.0, 'resolution', "float", "The angle between each laser in the sensor. Expressed in degrees in decimal format. (i. e.), half a degree is expressed as 0.5. Used when creating the arc object.")
    add_property('scan_window', 180.0, 'scan_window', "float", "The full angle covered by the sensor. Expressed in degrees in decimal format. Used when creating the arc object.")
    add_property('visible_arc', False, 'Visible_arc', "boolean", "if the laser arc should be displayed during the simulation")
    add_property('layers', 1, 'layers', "integer", "Number of scanning planes used by the sensor.")
    add_property('layer_separation', 0.8, 'layer_separation', "float", "The angular separation between the planes. Must be given in degrees.")
    add_property('layer_offset', 0.125, 'layer_offset', "float", "The horizontal distance between the scan points in consecutive scanning layers. Must be given in degrees.")

    def __init__(self, obj, parent=None):
        """
        Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        arc_prefix = 'Arc_'

        # Look for a child arc to use for the scans
        for child in obj.children:
            if arc_prefix in child.name:
                self._ray_arc = child
                logger.info("Sick: Using arc object: '{0}'".format(self._ray_arc))
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

            logger.debug("RAY %d = [%.4f, %.4f, %.4f]" % (vertex.index, self._ray_list[vertex.index-1][0],self._ray_list[vertex.index-1][1],self._ray_list[vertex.index-1][2]))

        # Get some information to be able to deform the arcs
        if self.visible_arc:
            layers = 1
            if 'layers' in self.blender_obj:
                layers = self.blender_obj['layers']
            self._vertex_per_layer = len(self._ray_list) // layers

        logger.info('Component initialized')


    def default_action(self):
        """
        Do ray tracing from the SICK object using a semicircle

        Generates a list of lists, with the points located.
        Also deforms the geometry of the arc associated to the SICK,
        as a way to display the results obtained.
        """
        #logger.debug("ARC POSITION: [%.4f, %.4f, %.4f]" % (self.blender_obj.position[0], self.blender_obj.position[1], self.blender_obj.position[2]))

        # Get the inverse of the transformation matrix
        inverse = self.position_3d.matrix.inverted()

        index = 0
        for ray in self._ray_list:
            # Transform the ray to the current position and rotation
            #  of the sensor
            correct_ray = self.position_3d.matrix * ray

            # Shoot a ray towards the target
            target,point,normal = self.blender_obj.rayCast(correct_ray,None,self.laser_range)
            #logger.debug("\tTarget, point, normal: {0}, {1}, {2}".format(target, point, normal))

            # Register when an intersection occurred
            if target:
                distance = self.blender_obj.getDistanceTo(point)
                # Return the point to the reference of the sensor
                new_point = inverse * point

                #logger.debug("\t\tGOT INTERSECTION WITH RAY: [%.4f, %.4f, %.4f]" % (correct_ray[0], correct_ray[1], correct_ray[2]))
                #logger.debug("\t\tINTERSECTION AT: [%.4f, %.4f, %.4f] = %s" % (point[0], point[1], point[2], target))
            # If there was no intersection, store the default values
            else:
                distance = self.laser_range
                new_point = [0.0, 0.0, 0.0]

            # Save the information gathered
            self.local_data['point_list'][index] = [new_point[0], new_point[1], new_point[2]]
            self.local_data['range_list'][index] = distance
            index += 1

        # Change the shape of the arc to show what the sensor detects
        if self.visible_arc:
            for mesh in self._ray_arc.meshes:
                for mat in range(mesh.numMaterials):
                    index = 0
                    for v_index in range(mesh.getVertexArrayLength(mat)):
                        # Switch to a new layer after a set number of vertices
                        if index % self._vertex_per_layer == 0:
                            index += 1

                        # Skip the first vertex of a tringle. It will always
                        #  be at the origin, and should not be changed
                        if v_index % 3 == 0:
                            continue

                        # Place the next vertex in the triangle
                        if v_index % 3 == 1:
                            vertex = mesh.getVertex(mat, v_index)
                            vertex.setXYZ(self.local_data['point_list'][index])

                        # Set the final vertex, in the correct order to have
                        #  the normals facing upwards.
                        if v_index % 3 == 2:
                            vertex = mesh.getVertex(mat, v_index)
                            vertex.setXYZ(self.local_data['point_list'][index-1])
                            index += 1
