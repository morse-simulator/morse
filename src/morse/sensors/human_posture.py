import math
import GameLogic
import morse.helpers.sensor

class HumanPostureClass(morse.helpers.sensor.MorseSensorClass):
    """ Class definition for the human posture exporter.
        This sensor exports the posture of a human model moving in the simulator.
        
        A human posture is a vector of 39 floats. The 3 first are the [x,y,z] 
        world position, the 3 next are the global rotation on the [x,y,z] axis.
        The 33 other one are described on a separate document (ask easisbot@laas.fr)
        
        Note: all angles are expressed in radians
        
        """

    def __init__(self, obj, parent=None):
        """ Constructor method.
            Receives the reference to the Blender object.
            The second parameter should be the name of the object's parent. """
        print("######## HUMAN POSTURE EXPORTER FOR '%s' INITIALIZING ########" % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self.local_data = {    'x': 0.0,
                            'y': 0.0,
                            'z':0.0,
                            'yaw': 0.0,
                            'pitch': 0.0,
                            'roll': 0.0,
                            'dof_13': 0.0,
                            'dof_14': 0.0,
                            'dof_15': 0.0,
                            'dof_16': 0.0,
                            'dof_17': 0.0,
                            'dof_18': 0.0,
                            'dof_19': 0.0,
                            'dof_20': 0.0,
                            'dof_21': 0.0,
                            'dof_22': 0.0,
                            'dof_23': 0.0,
                            'dof_24': 0.0,
                            'dof_25': 0.0,
                            'dof_26': 0.0,
                            'dof_27': 0.0,
                            'dof_28': 0.0,
                            'dof_29': 0.0,
                            'dof_30': 0.0,
                            'dof_31': 0.0,
                            'dof_32': 0.0,
                            'dof_33': 0.0,
                            'dof_34': 0.0,
                            'dof_35': 0.0,
                            'dof_36': 0.0,
                            'dof_37': 0.0,
                            'dof_38': 0.0,
                            'dof_39': 0.0,
                            'dof_40': 0.0,
                            'dof_41': 0.0,
                            'dof_42': 0.0,
                            'dof_43': 0.0,
                            'dof_44': 0.0,
                            'dof_45': 0.0}
        
        self.data_keys = self.local_data.keys()

        # Initialise the copy of the data
        for variable in self.data_keys:
            self.modified_data.append(self.local_data[variable])


        print ('######## HUMAN POSTURE EXPORTER INITIALIZED ########')


    def default_action(self):
        """ Extract the human posture

        """
        self.local_data['x'] = float(self.position_3d.x)
        self.local_data['y'] = float(self.position_3d.y)
        self.local_data['z'] = float(self.position_3d.z)
        self.local_data['yaw'] = float(self.position_3d.yaw)
        self.local_data['pitch'] = float(self.position_3d.pitch)
        self.local_data['roll'] = float(self.position_3d.roll)

