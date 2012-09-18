import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.sensor
import bpy

"""
Important note:

    The 'logger.debug' instructions take some processor work, even if they are
    not displayed. For this reason, it is best to comment out these lines in
    the 'default_action' method.
"""

class SICKClass(morse.core.sensor.MorseSensorClass):
    """ SICK laser range sensor """

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
                logger.info("Sick: Using arc object: '{0}'".format(self._ray_arc))
                break

        # Set its visibility, according to the settings
        self._visible = self.blender_obj['Visible_arc']
        self._ray_arc.setVisible(self._visible)
        self._ray_list = []

        # Create an empty list to store the intersection points
        self.local_data['point_list'] = []
        self.local_data['range_list'] = []

        # Get the datablock of the arc, to extract its vertices
        ray_object = bpy.data.objects[self._ray_arc.name]
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
        if self._visible:
            self._vertex_per_layer = len(self._ray_list) // self.blender_obj['layers']

        logger.info('Component initialized')


    def default_action(self):
        """ Do ray tracing from the SICK object using a semicircle

        Generates a list of lists, with the points located.
        Also deforms the geometry of the arc associated to the SICK,
        as a way to display the results obtained.
        """
        #logger.debug("=== NEW SCAN at time %s ===" % bge.logic.current_time)
        #logger.debug("ARC POSITION: [%.4f, %.4f, %.4f]" % (self.blender_obj.position[0], self.blender_obj.position[1], self.blender_obj.position[2]))

        # Get the inverse of the transformation matrix
        inverse = self.position_3d.matrix.inverted()

        index = 0
        for ray in self._ray_list:
            # Transform the ray to the current position and rotation
            #  of the sensor
            correct_ray = self.position_3d.matrix * ray

            # Shoot a ray towards the target
            target,point,normal = self.blender_obj.rayCast(correct_ray,None,self.blender_obj['laser_range'])
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
                distance = self.blender_obj['laser_range']
                new_point = [0.0, 0.0, 0.0]

            # Save the information gathered
            self.local_data['point_list'][index] = [new_point[0], new_point[1], new_point[2]]
            self.local_data['range_list'][index] = distance
            index += 1

        # Change the shape of the arc to show what the sensor detects
        if self._visible:
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
