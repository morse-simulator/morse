import GameLogic
if GameLogic.pythonVersion < 3:
    import Mathutils as mathutils
else:
    import mathutils
import morse.helpers.sensor
import morse.helpers.math

class SICKClass(morse.helpers.sensor.MorseSensorClass):
    """ SICK laser range sensor """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        print ("######## SICK '%s' INITIALIZING ########" % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        arc_prefix = 'Arc_'
        if GameLogic.pythonVersion < 3:
            arc_prefix = 'OB' + arc_prefix

        # Look for a child arc to use for the scans
        for child in obj.children:
            if arc_prefix in child.name:
                self.ray_arc = child
                print ("Sick: Using arc object: '{0}'".format(self.ray_arc))
                break

        # Set its visibility, according to the settings
        self.ray_arc.setVisible(self.blender_obj['Visible_arc'])

        # Create an empty list to store the intersection points
        self.local_data['point_list'] = []

        self.data_keys = ['point_list']

        # Initialise the copy of the data
        for variable in self.data_keys:
            self.modified_data.append(self.local_data[variable])

        print ('######## SICK INITIALIZED ########')


    def default_action(self):
        """ Do ray tracing from the SICK object using a semicircle

        Generates a list of lists, with the points located.
        """
        # Reset the list of points
        self.local_data['point_list'] = []

        # Obtain the rotation matrix of the sensor.
        inverted_matrix = morse.helpers.math.invert_rotation_matrix(self.blender_obj)

        # Create a vector for the mathutils operations
        vector_point = mathutils.Vector()

        # Get the mesh for the semicircle
        for mesh in self.ray_arc.meshes:
            for mat in range(mesh.numMaterials):
                for v_index in range(mesh.getVertexArrayLength(mat)):
                    vertex = mesh.getVertex(mat, v_index)
                    vertex_pos = vertex.getXYZ()

                    #print ("\tORIGINAL POINT: [%.4f, %.4f, %.4f]" % (vertex_pos[0], vertex_pos[1], vertex_pos[2]))

                    # Convert the vertex to a vector
                    fill_vector (vector_point, vertex_pos)

                    # Skip the center vertex
                    # NOTE: Make sure the center vertex of the arc
                    #  has local coordinates 0.0, 0.0, 0.0
                    if vector_point.length == 0:
                    #if vertex_pos == [0, 0, 0]:
                        #print ("\t\tskipping over center vertex")
                        continue

                    # Adjust the vector coordinates to the rotation
                    #  of the robot
                    corrected_vertex = self.blender_obj.getAxisVect(vector_point)

                    #print ("\tARC POSITION: [%.4f, %.4f, %.4f]" % (self.blender_obj.position[0], self.blender_obj.position[1], self.blender_obj.position[2]))

                    #ray = self.blender_obj.position
                    ray = [0, 0, 0]
                    # Displace according to the arc vertices
                    for i in range(3):
                        ray[i] = self.blender_obj.position[i] + corrected_vertex[i]

                    #print ("\tv: [%.2f, %.2f, %.2f]\t\tr: [%.2f, %.2f, %.2f]" % (vertex_pos[0], vertex_pos[1], vertex_pos[2], ray[0], ray[1], ray[2]))

                    # Shoot a ray towards the target
                    target,point,normal = self.blender_obj.rayCast(ray,None,self.blender_obj['laser_range'])
                    #print ("Target, point, normal: {0}, {1}, {2}".format(target, point, normal))

                    # If there was an intersection,
                    #  send the vertex to that point
                    if target:
                        #print ("\t\tGOT INTERSECTION WITH RAY: [%.4f, %.4f, %.4f]" % (ray[0], ray[1], ray[2]))
                        #print ("\t\tINTERSECTION AT: [%.4f, %.4f, %.4f] = %s" % (point[0], point[1], point[2], target))

                        # Substract the sensor coordinates
                        #  from the intersection point
                        for i in range(3):
                            point[i] = point[i] - self.blender_obj.position[i]
                        #print ("\t\tARC POINT: [%.4f, %.4f, %.4f]" % (point[0], point[1], point[2]))

                        # Create a vector object
                        fill_vector (vector_point, point)

                        # Multiply the resulting point by the inverse
                        #  of the sensor rotation matrix
                        arc_point = vector_point * inverted_matrix
                        #print ("\t\tARC POINT: [%.4f, %.4f, %.4f]" % (arc_point[0], arc_point[1], arc_point[2]))

                        # Do not move the point if the ray intersection
                        #  happened at the origin
                        #  (because this breaks the arc and makes all
                        #  subsequent rays wrong)
                        if valid_range (arc_point, 0.1):
                            # Send the vertex to the new location
                            vertex.setXYZ(arc_point)

                        # Convert the arc point from a vector to a list
                        arc_point = [arc_point[0], arc_point[1], arc_point[2]]

                    # Otherwise return the vertex to its original position
                    else:
                        # Create a vector object
                        fill_vector (vector_point, vertex_pos)
                        # Give it the correct size
                        vector_point.normalize()
                        vector_point = vector_point * self.blender_obj['laser_range']

                        # Move the vertex to the computed position
                        vertex.setXYZ(vector_point)
                        #print ("\t\tNO intersection. [%.4f, %.4f, %.4f]" % (vector_point[0], vector_point[1], vector_point[2]))

                        # Add a point at 0,0,0 to the output file,
                        #  to mark that this ray did not find anything
                        arc_point = [0.0, 0.0, 0.0]

                    self.local_data['point_list'].append(arc_point)


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


def fill_vector(vector, point):
    """ Copy the contents of a list into an existing vector structure. """
    for i in range(3):
        vector[i] = point[i]
