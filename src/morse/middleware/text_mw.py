import logging; logger = logging.getLogger("morse." + __name__)
import bge
import re
import sys
import morse.core.middleware

class TextOutClass(morse.core.middleware.MorseMiddlewareClass):
    """ Produce text files as output for the components """

    def __init__(self):
        # Call the constructor of the parent class
        super(self.__class__,self).__init__()

        self._file_list = dict()
        self._index_list = dict()

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

        # Open the file
        file_name = '{0}_{1}.txt'.format(parent_name, component_name)
        FILE = open(file_name, 'wb')
        self._file_list[component_name] = FILE
        self._index_list[component_name] = 1
        logger.info("File: '%s' opened for writing" % file_name)

        # Extract the information for this middleware
        # This will be tailored for each middleware according to its needs
        function_name = mw_data[1]

        function = self._check_function_exists(function_name)

        if function != None:
            # The function exists within this class,
            #  so it can be directly assigned to the instance
            if function_name == "write_data":
                component_instance.output_functions.append(function)
            # Data write functions
            elif function_name == "post_message":
                component_instance.output_functions.append(function)

            else:
                # Get the reference to the external module
                # (should be already included)
                source_file = mw_data[2]
                module_name = re.sub('/', '.', source_file)
                module = sys.modules[module_name]
                try:
                    # Call the init method of the new serialisation
                    # Sends the name of the function as a means to identify
                    #  what kind of port it should use.
                    module.init_extra_module(self, component_instance, function, mw_data)
                except AttributeError as detail:
                    logger.error("%s in module '%s'" % (detail, source_file))
                return

            # Prepare a list with the data for the header of the file
            data = []
            data.append("ROBOT %s || SENSOR %s\n" % (parent_name, component_name))
            data.append("(distance, globalVector(3), localVector(3))\n")
            data.append(repr(component_instance.relative_position) + "\n\n")
            # Write the header
            for line in data:
                FILE.write(line.encode())

        else:
            # If there is no such function in this module,
            #  try importing from another one
            try:
                # Insert the method in this class
                function = self._add_method(mw_data, component_instance)

            except IndexError as detail:
                logger.error("Method '%s' is not known, and no external module has been specified. Check the 'component_config.py' file for typos" % function_name)
                return
                    
    def post_message(self, component_instance):
        """ Dummy function to call the real storage method """
        self.write_data(component_instance)


    def write_data(self, component_instance):
        """ Write the current data to the adequate file

        The argument is a copy of the component instance.
        """
        parent_position = component_instance.robot_parent.position_3d
        FILE = self._file_list[component_instance.blender_obj.name]
        line = "==> Data at X,Y,Z: [%.6f %.6f %.6f] yaw,pitch,roll: [%.6f %.6f %.6f] | index %d | time %.2f\n" % (parent_position.x, parent_position.y, parent_position.z, parent_position.yaw, parent_position.pitch, parent_position.roll, self._index_list[component_instance.blender_obj.name], bge.logic.current_time)
        self._index_list[component_instance.blender_obj.name] += 1
        FILE.write(line.encode())
        for variable, data in component_instance.local_data.items():
            if isinstance(data, float):
                line = "\t%s = %.6f\n" % (variable, data)
            else:
                line = "\t%s = %s\n" % (variable, repr(data))
            FILE.write(line.encode())
