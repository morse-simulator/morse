import logging; logger = logging.getLogger("morse." + __name__)
import bge
import mathutils
import math
import morse.core.sensor
import morse.helpers.morse_math

class VelodyneClass(morse.core.sensor.MorseSensorClass):
    """ Velodyne laser range sensor """

    def __init__(self, obj, parent=None):
        """ Constructor method.

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
                logger.info("Velodyne: Using arc object: '{0}'".format(self._ray_arc))
                break

        # Set its visibility, according to the settings
        self._ray_arc.setVisible(self.blender_obj['Visible_arc'])
        self._ray_list = []

        # Create an empty list to store the intersection points
        self.local_data['point_list'] = []
        self.local_data['range_list'] = []

        # Initialize the ray vectors and the point list
        for mesh in self._ray_arc.meshes:
            for mat in range(mesh.numMaterials):
                index = 0
                for v_index in range(mesh.getVertexArrayLength(mat)):
                    vertex = mesh.getVertex(mat, v_index)
                    vertex_pos = vertex.getXYZ()

                    # Create a vector for the mathutils operations
                    vector_point = mathutils.Vector()

                    # Convert the vertex to a vector
                    morse.helpers.morse_math.fill_vector (vector_point, vertex_pos)

                    # Skip the center vertex
                    # NOTE: Make sure the center vertex of the arc
                    #  has local coordinates 0.0, 0.0, 0.0
                    if vector_point.length == 0:
                        logger.debug("Center vertex has index: %d" % index)
                        continue

                    # Insert empty points into the data list
                    self.local_data['point_list'].append([0.0, 0.0, 0.0])
                    # Insert zeros into the range list
                    self.local_data['range_list'].append(0.0)
                    # Insert the coordinates of the ray
                    self._ray_list.append(vector_point)
                    logger.debug("RAY %d = [%.4f, %.4f, %.4f]" % (index, self._ray_list[index][0],self._ray_list[index][1],self._ray_list[index][2]))

                    index = index + 1

                logger.info("Velodyne has %d rays" % len(self._ray_list))

        logger.info('Component initialized')


    def default_action(self):
        """ Do ray tracing from the Velodyne object using a semicircle

        Generates a list of lists, with the points located.
        Also deforms the geometry of the arc associated to the Velodyne,
        as a way to display the results obtained.
        """
        # Obtain the rotation matrix of the sensor.
        robot_inverted_matrix = morse.helpers.morse_math.invert_rotation_matrix(self.robot_parent.blender_obj)
        sensor_inverted_matrix = morse.helpers.morse_math.invert_rotation_matrix(self.blender_obj)

        # Create a vector for the mathutils operations
        vector_point = mathutils.Vector()

        logger.debug("=== NEW SCAN at time %s ===" % bge.logic.current_time)
        logger.debug("ARC POSITION: [%.4f, %.4f, %.4f]" % (self.blender_obj.position[0], self.blender_obj.position[1], self.blender_obj.position[2]))
        # Get the mesh for the semicircle
        for mesh in self._ray_arc.meshes:
            for mat in range(mesh.numMaterials):
                index = 0
                for v_index in range(mesh.getVertexArrayLength(mat)):
                    vertex = mesh.getVertex(mat, v_index)
                    vertex_pos = vertex.getXYZ()

                    # Convert the vertex to a vector
                    morse.helpers.morse_math.fill_vector (vector_point, vertex_pos)

                    # Skip the center vertex
                    # NOTE: Make sure the center vertex of the arc
                    #  has local coordinates 0.0, 0.0, 0.0
                    if vector_point.length == 0:
                        continue

                    base_ray = self._ray_list[index]
                    # Adjust the vector coordinates to the rotation
                    #  of the robot
                    corrected_ray = self.blender_obj.getAxisVect(base_ray)

                    ray = [0, 0, 0]
                    # Displace according to the arc vertices
                    for i in range(3):
                        ray[i] = self.blender_obj.position[i] + corrected_ray[i]

                    logger.debug("\t%d: base_ray: [%.2f, %.2f, %.2f]\tray: [%.2f, %.2f, %.2f]" % (index, base_ray[0], base_ray[1], base_ray[2], ray[0], ray[1], ray[2]))

                    # Shoot a ray towards the target
                    target,point,normal = self.blender_obj.rayCast(ray,None,self.blender_obj['laser_range'])
                    logger.debug("\tTarget, point, normal: {0}, {1}, {2}".format(target, point, normal))

                    # If there was an intersection,
                    #  send the vertex to that point
                    if target:
                        logger.debug("\t\tGOT INTERSECTION WITH RAY: [%.4f, %.4f, %.4f]" % (ray[0], ray[1], ray[2]))
                        logger.debug("\t\tINTERSECTION AT: [%.4f, %.4f, %.4f] = %s" % (point[0], point[1], point[2], target))

                        # Substract the sensor coordinates
                        #  from the intersection point
                        for i in range(3):
                            point[i] = point[i] - self.blender_obj.position[i]
                        logger.debug("\t\tARC POINT: [%.4f, %.4f, %.4f]" % (point[0], point[1], point[2]))

                        # Create a vector object
                        morse.helpers.morse_math.fill_vector (vector_point, point)

                        # Multiply the resulting point by the inverse
                        #  of the sensor rotation matrix
                        arc_point = robot_inverted_matrix * vector_point
                        logger.debug("\t\tARC POINT: [%.4f, %.4f, %.4f]" % (arc_point[0], arc_point[1], arc_point[2]))

                        # Do not move the point if the ray intersection
                        #  happened at the origin
                        #  (because this breaks the arc and makes all
                        #  subsequent rays wrong)
                        if valid_range (arc_point, 0.1):
                            # Send the vertex to the new location
                            geometry_point = vector_point * sensor_inverted_matrix
                            vertex.setXYZ(geometry_point)

                        # Convert the arc point from a vector to a list
                        arc_point = [arc_point[0], arc_point[1], arc_point[2]]

                    # Otherwise return the vertex to its original position
                    else:
                        # Create a vector object
                        morse.helpers.morse_math.fill_vector (vector_point, base_ray)
                        # Give it the correct size
                        vector_point.normalize()
                        vector_point = vector_point * self.blender_obj['laser_range']

                        # Move the vertex to the computed position
                        vertex.setXYZ(vector_point)
                        logger.debug("\t\tNO intersection. [%.4f, %.4f, %.4f]" % (vector_point[0], vector_point[1], vector_point[2]))

                        # Add a point at 0,0,0 to the output file,
                        #  to mark that this ray did not find anything
                        arc_point = [0.0, 0.0, 0.0]

                    
                    #calculate ranges of the laserscanner based on Blender_object pose and points
                    xx = arc_point[0] - self.blender_obj.position[0]
                    yy = arc_point[1] - self.blender_obj.position[1]
                    self.local_data['range_list'][index] = math.sqrt(pow(xx,2)+pow(yy,2))
                    self.local_data['point_list'][index] = arc_point
                    index = index + 1


def valid_range(point_vector, radius):
    """ Determine if a ray is longer than radius

    A ray intersection will only be valid if it happens
    outside of a certain radius from the source.
    This radius should be equivalent to the size of
    the laser emiter.
    """
    if point_vector.length < radius:
        return False
    else:
        return True
