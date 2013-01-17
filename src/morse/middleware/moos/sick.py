import logging; logger = logging.getLogger("morse." + __name__)
import pymoos.MOOSCommClient
import morse.core.datastream
from morse.core import blenderapi

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    # Compose the name of the port, based on the parent and module names
    component_name = component_instance.bge_object.name
    parent_name = component_instance.robot_parent.bge_object.name

     # Add the new method to the component
    component_instance.output_functions.append(function)

    # post lidar settings to the database only once at startup
    curTime=blenderapi.persistantstorage().current_time
    self.m.Notify('sScanAngle',component_instance.bge_object['scan_window'],curTime) 
    self.m.Notify('sScanResolution',component_instance.bge_object['resolution'],curTime) 
    self.m.Notify('sScanRange',component_instance.bge_object['laser_range'],curTime)

    # Generate one publisher and one topic for each component that is a sensor and uses post_message
    logger.info('######## SICK LIDAR-SENSOR INITIALIZED ########')

def post_2DLaserScan(self, component_instance):
    """ Publish the data on the rostopic
		"""
    curTime=pymoos.MOOSCommClient.MOOSTime()
    num_readings = component_instance.bge_object['scan_window'] / component_instance.bge_object['resolution']

    # build string of the form: '[1x180]{4.9,29.2,...,2.98}'
    # replace [] in python string conversion to {} expected by MOOS parsing code
    laserscan = str(component_instance.local_data['range_list']).replace('[','{').replace(']','}')
    # add array size to beginning of string
    #laserscan = '[1x' + '{0:.0}'.format(num_readings) + ']'+laserscan
    laserscan = '[1x' + '%.0f' % num_readings + ']'+laserscan

    parent_name = component_instance.robot_parent.bge_object.name
    self.m.Notify('zLidarDist',laserscan,curTime)

