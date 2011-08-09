import logging; logger = logging.getLogger("morse." + __name__)
import GameLogic
import morse.core.middleware

class TextOutClass(morse.core.middleware.MorseMiddlewareClass):
    """ Produce text files as output for the components """

    def __init__(self, obj, parent=None):
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self._file_list = dict()

    def __del__(self):
        """ Close all opened files """
        for component_name, file in self._file_list.items():
            file.close()

    def register_component(self, component_name, component_instance, mw_data):
        """ Open a text file to write the data

        The name of the file is composed of the robot and sensor names.
        Only useful for sensors.
        """
        parent_name = component_instance.robot_parent.blender_obj.name

        # Extract the information for this middleware
        # This will be tailored for each middleware according to its needs
        function_name = mw_data[1]

        try:
            # Get the reference to the function
            function = getattr(self, function_name)
        except AttributeError as detail:
            logger.error("%s. Check the 'component_config.py' file for typos" % detail)
            return

        # Data write functions
        if function_name == "write_data":
            component_instance.output_functions.append(function)

        # Prepare a list with the data for the header of the file
        data = []
        data.append("ROBOT %s || SENSOR %s\n" % (parent_name, component_name))
        data.append("(distance, globalVector(3), localVector(3))\n")
        data.append(repr(component_instance.relative_position) + "\n")

        # Open the file and write a header
        file_name = '{0}_{1}.txt'.format(parent_name, component_name)
        FILE = open(file_name, 'wb')
        for line in data:
            FILE.write(line.encode())
        self._file_list[component_name] = FILE
        logger.info("File: '%s' opened for writing" % file_name)


    def write_data(self, component_instance):
        """ Write the current data to the adequate file

        The argument is a copy of the component instance.
        """
        parent_position = component_instance.robot_parent.position_3d
        FILE = self._file_list[component_instance.blender_obj.name]
        line = "==> Data at X,Y,Z: [%.6f %.6f %.6f] yaw,pitch,roll: [%.6f %.6f %.6f] | time %s\n" % (parent_position.x, parent_position.y, parent_position.z, parent_position.yaw, parent_position.pitch, parent_position.roll, GameLogic.current_time)
        FILE.write(line.encode())
        for variable, data in component_instance.local_data.items():
            line = "\t%s = %s\n" % (variable, repr(data))
            FILE.write(line.encode())
