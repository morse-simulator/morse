import morse.builder.creator
import morse.builder.blenderobjects

class ROS(morse.builder.creator.MiddlewareCreator):
  def __init__(self, name="ROS_Empty"):
    morse.builder.creator.MiddlewareCreator.__init__(self, name, 
      "morse/middleware/ros_mw", "ROSClass", "ros_empty")

