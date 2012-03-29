#--- ### Header
bl_info = {
    "name": "MORSE import robot path (.pth)",
    "author": "Gilberto Echeverria",
    "version": (1, 0, 0),
    "blender": (2, 6, 0),
    "api": 41092,
    "location": "File>Import-Export",
    "category": "MORSE",
    #"category": "Import-Export",
    "description": "Load a 2D path stored as a set of X, Y, yaw",
    "warning": "",
    "wiki_url": "",
    "tracker_url": "https://softs.laas.fr/bugzilla/"
}

import bpy


class MorseLoadTextPath(bpy.types.Operator):
    """ Load the path followed by a robot in MORSE
    The input is a text file with three floats per line: X, Y, yaw"
    """
    bl_idname = "import_mesh.morse_path"
    bl_label = "Import robot path (.pth)"
    bl_description = "Load the path followed by a robot in MORSE"

    segment_counter = 1
    mat = None
    origin_object = None

    filepath  = bpy.props.StringProperty(subtype='FILE_PATH')
    use_origin = bpy.props.BoolProperty(name="Snap to selected object", description="Snap the path to the currently selected object (location and rotation)", default=True)
    path_color = bpy.props.FloatVectorProperty(subtype='COLOR', default=(0.7, 0.1, 0.0), description="Path color")

    @classmethod
    def poll(self, context):
        return (context.mode == 'OBJECT')

    def execute(self, context):
        """ Processes a click on the "submit" button in the filebrowser window '"""
        self.init_material()
        print("Input path file is: '%s'" % self.filepath)
        self.read_morse_path(self.filepath)
        return {'FINISHED'}

    def invoke(self, context, event):
        """ Processes the call to the path importer from the menu.
        Opens a filebrowser window
        """
        wm = context.window_manager
        wm.fileselect_add(self)

        # Place the path at the currently selected object.
        # If there are more than one selected, use the active one
        if self.use_origin and bpy.context.selected_objects != []:
            self.origin_object = bpy.context.active_object
            print ("Using the object '%s' as the origin of the path" % self.origin_object)
            
        return {'RUNNING_MODAL'}

    def draw(self, context):
        """ Define the menu to be shown when loading a path file """
        layout = self.layout

        layout.prop(self, "use_origin")
        box = layout.box()
        box.label(text="Path color:")
        box.prop(self, "path_color")


    def new_segment(self, location, orientation):
        """ Add a new segment to draw a path

        :param location: 3D coordinates of the waypoint
        :param orientation: Euler array for the orientation of the segment
        """
        # Give a name to the new segment
        new_name = "Segment%d" % self.segment_counter
        self.segment_counter += 1

        # Define the coordinates of the vertices.
        #  Each vertex is defined by 3 consecutive floats.
        coords=[(-0.5, -0.1, -0.1), (-0.5, 0.1, -0.1), (-0.5, 0.1 ,0.1), \
        (-0.5, -0.1, 0.1), (0.5, 0.0, 0.0)]

        # Define the faces by index numbers.
        #  Each faces is defined by 3 or 4 consecutive integers.
        faces=[ (2,1,0,3), (0,1,4), (1,2,4), (2,3,4), (3,0,4)]

        # Create the mesh
        mesh = bpy.data.meshes.new(new_name+"Mesh")
        # Add the new material
        mesh.materials.append(self.mat)
        # Create an object with that mesh
        obj = bpy.data.objects.new(new_name, mesh)
        # Set physics simulation to no collision
        obj.game.physics_type = 'NO_COLLISION'
        # Position and orient the object using the parameters provided
        obj.location = location
        obj.rotation_euler = orientation
        # Link object to scene
        bpy.context.scene.objects.link(obj)

        # Fill the mesh with verts, edges, faces
        # edges or faces should be [], or you ask for problems
        mesh.from_pydata(coords,[],faces)
        # Update mesh with new data
        mesh.update(calc_edges=True)


    def init_material(self):
        """ Create a new material for the segments """
        self.mat = bpy.data.materials.new("PathMat")
        self.mat.diffuse_color = self.path_color
        # Disable the shadows on the path object
        self.mat.use_shadows = False
        self.mat.use_cast_buffer_shadows = False


    def read_morse_path(self, filename):
        """ Get the position and orientation of a robot trajectory

        Read the data from a file in the .pth format (3 floats for X, Y, yaw)
        :param filename: name of the path file
        """
        # Open the file
        fp = open(filename, "r")
        for line in fp:
            # Extract the position of the robot
            coords = line.split()
            try:
                segment_position = [float(coords[0]), float(coords[1]), 0.0]
                segment_orientation = [0.0, 0.0, float(coords[2])]
            except ValueError as detail:
                print ("Invalid token in file '%s': %s" % (filename, detail))
                print ("Aborting import")
                return

            self.new_segment(segment_position, segment_orientation)

        # A trick to make the selection and join correctly
        # First select one of the segments. This will be the Active Object
        active_segment = bpy.data.objects["Segment1"]
        active_segment.select = True
        bpy.context.scene.objects.active = active_segment
        # Then select all the rest. This does not change the Active Object
        bpy.ops.object.select_pattern(pattern="Segment*")
        # Join. The active Object must be selected
        bpy.ops.object.join()

        # Change the name of the object
        obj = bpy.context.active_object
        obj.name = "Path"

        # Locate the path if there is a selected object
        if self.origin_object != None:
            obj.location = self.origin_object.location
            obj.rotation_euler = self.origin_object.rotation_euler


def menu_draw(self, context):
    self.layout.operator_context = 'INVOKE_REGION_WIN'
    self.layout.operator(MorseLoadTextPath.bl_idname, "Load MORSE path (.pth)")


#--- ### Register
def register():
    bpy.utils.register_module(__name__)
    bpy.types.INFO_MT_file_import.prepend(menu_draw)
def unregister():
    bpy.types.INFO_MT_file_import.remove(menu_draw)
    bpy.utils.unregister_module(__name__)

#--- ### Main code
if __name__ == '__main__':
    register()
