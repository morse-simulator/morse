import mathutils
import math
import morse.core.sensor
import morse.helpers.math

"""
Morse is MW agnostic, but as help, I base this sensor on:
http://www.ros.org/doc/api/sensor_msgs/html/msg/Range.html
the Range message of ROS middleware

based on sick laser, with a range of 20 deg.
"""

class InfraRedClass(morse.core.sensor.MorseSensorClass):
    """ SICK laser range sensor """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        print ("######## InfraRed '%s' INITIALIZING ########" % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        arc_prefix = 'Arc_'

        # Look for a child arc to use for the scans
        for child in obj.children:
            if arc_prefix in child.name:
                self._ray_arc = child
                print ("Sick: Using arc object: '{0}'".format(self._ray_arc))
                break

        # Set its visibility, according to the settings
        self._ray_arc.setVisible(self.blender_obj['Visible_arc'])
        self._ray_list = []

        self.local_data['range'] = -1

        # Initialize the ray vectors and the point list
        for mesh in self._ray_arc.meshes:
            for mat in range(mesh.numMaterials):
                for v_index in range(mesh.getVertexArrayLength(mat)):
                    vertex = mesh.getVertex(mat, v_index)
                    vertex_pos = vertex.getXYZ()

                    # Create a vector for the mathutils operations
                    vector_point = mathutils.Vector()

                    # Convert the vertex to a vector
                    fill_vector (vector_point, vertex_pos)

                    # Skip the center vertex
                    # NOTE: Make sure the center vertex of the arc
                    #  has local coordinates 0.0, 0.0, 0.0
                    if vector_point.length == 0:
                        continue

                    # Insert the coordinates of the ray
                    self._ray_list.append(vector_point)

        print ('######## SICK INITIALIZED ########')


    def default_action(self):
        """ Do ray tracing from the SICK object using a semicircle

        Generates a list of lists, with the points located.
        Also deforms the geometry of the arc associated to the SICK,
        as a way to display the results obtained.
        """
        # Obtain the rotation matrix of the sensor.
        inverted_matrix = morse.helpers.math.invert_rotation_matrix(self.blender_obj)

        # Create a vector for the mathutils operations
        vector_point = mathutils.Vector()

        # init range to infinite (-1)
        self.local_data['range'] = -1.0

        # Get the mesh for the semicircle
        for mesh in self._ray_arc.meshes:
            for mat in range(mesh.numMaterials):
                index = 0
                for v_index in range(mesh.getVertexArrayLength(mat)):
                    vertex = mesh.getVertex(mat, v_index)
                    vertex_pos = vertex.getXYZ()

                    # Convert the vertex to a vector
                    fill_vector (vector_point, vertex_pos)

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

                    # Shoot a ray towards the target
                    target,point,normal = self.blender_obj.rayCast(ray,None,self.blender_obj['ir_range'])

                    # If there was an intersection,
                    #  send the vertex to that point
                    if target:
                        # Substract the sensor coordinates
                        #  from the intersection point
                        for i in range(3):
                            point[i] = point[i] - self.blender_obj.position[i]

                        # Create a vector object
                        fill_vector (vector_point, point)

                        # Multiply the resulting point by the inverse
                        #  of the sensor rotation matrix
                        arc_point = vector_point * inverted_matrix

                        # Do not move the point if the ray intersection
                        #  happened at the origin
                        #  (because this breaks the arc and makes all
                        #  subsequent rays wrong)
                        if valid_range (arc_point, 0.1):
                            # Send the vertex to the new location
                            vertex.setXYZ(arc_point)

                        #calculate ranges of the laserscanner based on Blender_object pose and points
                        distance = math.sqrt(pow(arc_point[0],2)+pow(arc_point[1],2))

                        # Distance is 0 when maxrange, set to maxrange in range-array
                        if (distance > 0.0) and (distance < self.blender_obj['ir_range']) and ((distance < self.local_data['range']) or (self.local_data['range'] == -1)):
                            self.local_data['range'] = distance

                    # Otherwise return the vertex to its original position
                    else:
                        # Create a vector object
                        fill_vector (vector_point, base_ray)
                        # Give it the correct size
                        vector_point.normalize()
                        vector_point = vector_point * self.blender_obj['ir_range']

                        # Move the vertex to the computed position
                        vertex.setXYZ(vector_point)

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


def fill_vector(vector, point):
    """ Copy the contents of a list into an existing vector structure. """
    for i in range(3):
        vector[i] = point[i]

