import logging; logger = logging.getLogger("morse." + __name__)
import mathutils
import math
import morse.core.sensor
import morse.helpers.math
import bge
import bpy


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

        ray_object = bpy.data.objects[self._ray_arc.name]
        for vertex in ray_object.data.vertices:
            print ("Vertex %d = %s" % (vertex.index, vertex.co))

            if vertex.index == 0:
                continue

            # Get the vertices from the mesh
            #vertex_pos = vertex.co
            # Create a vector with the direction of the vertex
            #vector_point = mathutils.Vector(vertex_pos)
            # Store the vector in a list
            self._ray_list.append(vertex.co)

            # Insert empty points into the data list
            self.local_data['point_list'].append([0.0, 0.0, 0.0])
            # Insert zeros into the range list
            self.local_data['range_list'].append(0.0)

            logger.debug("RAY %d = [%.4f, %.4f, %.4f]" % (vertex.index, self._ray_list[vertex.index-1][0],self._ray_list[vertex.index-1][1],self._ray_list[vertex.index-1][2]))



        """
        # Initialize the ray vectors and the point list
        for mesh in self._ray_arc.meshes:
            for mat in range(mesh.numMaterials):
                index = 0
                for v_index in range(mesh.getVertexArrayLength(mat)):
                    # Skip the center vertex
                    # NOTE: It should always be the first vertex,
                    #  as created by the Builder script
                    if v_index == 0:
                        continue

                    # Get the vertices from the mesh
                    vertex = mesh.getVertex(mat, v_index)
                    vertex_pos = vertex.getXYZ()
                    # Create a vector with the direction of the vertex
                    vector_point = mathutils.Vector(vertex_pos)
                    # Store the vector in a list
                    self._ray_list.append(vector_point)

                    # Insert empty points into the data list
                    self.local_data['point_list'].append([0.0, 0.0, 0.0])
                    # Insert zeros into the range list
                    self.local_data['range_list'].append(0.0)

                    logger.debug("RAY %d = [%.4f, %.4f, %.4f]" % (index, self._ray_list[index][0],self._ray_list[index][1],self._ray_list[index][2]))

                    index = index + 1
        """

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

        """
        # Change the shape of the arc to show what the sensor detects
        if self._visible:
            for mesh in self._ray_arc.meshes:
                for mat in range(mesh.numMaterials):
                    index = 0
                    for v_index in range(mesh.getVertexArrayLength(mat)):
                        # Skip the center vertex
                        # NOTE: It should always be the first vertex,
                        #  as created by the Builder script
                        if v_index == 0:
                            continue

                        vertex = mesh.getVertex(mat, v_index)
                        vertex.setXYZ(self.local_data['point_list'][index])
                        index += 1
        """
