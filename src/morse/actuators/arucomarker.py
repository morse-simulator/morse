import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.actuator
from morse.core.services import service, async_service, interruptible
from morse.core import status
from morse.helpers.components import add_data, add_property
import mathutils
import math

class Arucomarker(morse.core.actuator.Actuator):
    """ 
    A virtual representation of the ArUco Marker
    http://www.uco.es/investiga/grupos/ava/node/26        
    """

    _name = "ArUco Marker"
    _short_desc = """
                  The ArUco marker is an AR-Marker that allows to compute the camera pose 
                  from images in the 'real world'. 
                  See: http://www.uco.es/investiga/grupos/ava/node/26
                  The purpose of this actuator is to  provide a virtual instance of such a 
                  marker in MORSE. By adding an ArUco marker to a MORSE simulation you can 
                  subsequently stream/export a (virtual) camera image and eventually use an 
                  AR Marker without a physical camera setup or, i.e, test algorithms or simulate 
                  visual servoring.

                  .. example::
                      ### Add a virtual ArUco marker to the scene
                      robot = ATRV()

                      aruco = Arucomarker()	
                      aruco.add_stream('ros', topic="/aruco_pose")
                      aruco.properties(zoffset=0.3, xoffset=-0.09, yoffset=.0)

                      robot.append(aruco) 
  
                  """


    add_data('x',     0.0, 'float', 'X axis translation metres')
    add_data('y',     0.0, 'float', 'Y axis translation metres')
    add_data('z',     0.0, 'float', 'Z axis translation metres')
    add_data('roll',  0.0, 'float', 'X axis rotation in rad')
    add_data('pitch', 0.0, 'float', 'Y axis rotation in rad')
    add_data('yaw',   0.0, 'float', 'Z axis rotation in rad')

    """
    Initialises translation properties, they can be accessed via builder script
    These properties add a static offset to the marker. You may want to use this
    if you plan on aligning the marker to a virtual camera, which would allow you
    to test your tracking algorithms based on the image of a virtual camera for
    instance
    """
    add_property('_xoffset', 0.0, 'xoffset', 'float', "X axis translation offset in metres")
    add_property('_yoffset', 0.0, 'yoffset', 'float', "Y axis translation offset in metres")
    add_property('_zoffset', 0.0, 'zoffset', 'float', "Z axis translation offset in metres")    

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        morse.core.actuator.Actuator.__init__(self, obj, parent)
        
        """ 
        Save the ArUco object and its parent to compute the relative
        position later on in default_action()
        """
        self.aruco = {}
        self.aruco['aruco']  = self.bge_object
        self.aruco['parent'] = self.robot_parent

        """ 
        Spawn the marker as specified in the builder script via translate
        The rotation is zeroed initially
        """
        self.local_data['x']     = self.aruco['aruco'].worldPosition[0] 
        self.local_data['y']     = self.aruco['aruco'].worldPosition[1] 
        self.local_data['z']     = self.aruco['aruco'].worldPosition[2]
        self.local_data['roll']  = 0.0 
        self.local_data['pitch'] = 0.0
        self.local_data['yaw']   = 0.0
        
        logger.info('Component initialized')

    @service
    def get_local_position(self):
        return self.aruco['aruco'].localPosition

    @service
    def get_world_position(self):
        return self.aruco['aruco'].worldPosition
    
    @service 
    def get_local_orientation(self):
        return self.aruco['aruco'].localOrientation

    @service
    def get_world_orientation(self):
        return self.aruco['aruco'].worldOrientation
    
    """ 
    Apply roation and translation, function 
    lifted from teleport actuator class +
    applying parents position in case it moves
    """
    def force_pose(self, position, orientation):
        me = self.aruco['aruco']
        parent = self.aruco['parent']
        pose3d = parent.position_3d
        parent_pose = mathutils.Vector((pose3d.x, pose3d.y, pose3d.z))
        if position:
            me.worldPosition = position + parent_pose
        if orientation:
            me.worldOrientation = orientation
    
    """ 
    The default action which is executed every LOGIC TICK
    Compute current location, i.e., provided via middleware (in metres)
    NOTE: By default, the ArUco translation (AT), as defined in the
    original ArUco library, corresponds to the MORSE translation 
    (MT) as follows:
    
    MTx =  ATz
    MTy =  ATx
    MTz = -ATy
 
    See example below
    """
    def default_action(self):
        position = mathutils.Vector( (self.local_data['z']+self._xoffset,
                                     self.local_data['x']+self._yoffset,
                                     (-1.0*self.local_data['y']+self._zoffset)) )

        orientation = mathutils.Euler([ self.local_data['roll' ],
                                        self.local_data['pitch'],
                                        self.local_data['yaw'  ] ])
       
        """ Convert Euler to Matrix, worldOrientation accepts Quat, Mat """
        orientation.order = "YZX"
        orientation_mat = orientation.to_matrix()
        self.force_pose(position, orientation_mat)
