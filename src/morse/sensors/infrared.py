import logging; logger = logging.getLogger("morse." + __name__)
import bge
import morse.core.sensor

class InfraredClass(morse.core.sensor.MorseSensorClass):
    """ Infrared radar sensor """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self.local_data['range'] = -1

        logger.info('Component initialized')


    def default_action(self):
        """ get the object-list scanned by the radar, and get their distance.
        In order to be detected by the radar, 
        the object must be Actor (checkbox in Physics properties)
        
        http://www.blender.org/documentation/blender_python_api_2_59_release/bge.types.html#bge.types.KX_GameObject.getDistanceTo
        http://www.blender.org/documentation/blender_python_api_2_59_release/bge.types.html#bge.types.KX_TouchSensor.hitObjectList
        """
        # get the controller thats running this python script
        cont = bge.logic.getCurrentController()
        # get the game object this controller is on
        infrared = cont.owner
        # get the radar sensor
        radar = cont.sensors['Radar']
        tmp = self.blender_obj['ir_range']

        for obj in radar.hitObjectList:
            # the robot is always in the list, even if the sensor is in front
            if obj.name != self.robot_parent.name():
                dist = infrared.getDistanceTo(obj)
                if dist < tmp:
                    tmp = dist
        self.local_data['range'] = tmp

