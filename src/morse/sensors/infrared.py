import logging; logger = logging.getLogger("morse." + __name__)
import bge
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
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self.local_data['range'] = -1

        logger.info('Component initialized')


    def default_action(self):
        """ get the object-list scanned by the radar, and get their distance
        
        http://www.blender.org/documentation/blender_python_api_2_58_release/bge.types.html#bge.types.KX_GameObject.getDistanceTo
        """
        # get the controller thats running this python script
        cont = bge.logic.getCurrentController()
        # get the game object this controller is on
        infrared = cont.owner
        # get the radar sensor
        #radar = infrared.sensors['Radar']
        radar = cont.sensors['Radar']
        tmp = self.blender_obj['ir_range']
        min_dist = .1
        for obj in radar.hitObjectList:
            dist = infrared.getDistanceTo(obj)
            if dist > min_dist and dist < tmp:
                tmp = dist
        self.local_data['range'] = tmp

