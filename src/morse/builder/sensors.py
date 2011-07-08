import morse.builder.creator

class GPS(morse.builder.creator.SensorCreator):
  def __init__(self, name="GPS"):
    morse.builder.creator.SensorCreator.__init__(self, name, 
      "morse/sensors/gps", "GPSClass", "morse_GPS")
    mesh = morse.builder.creator.Cylinder("GPSCylinder")
    mesh.scale = (.1,.1,.05)
    self.append(mesh)

class Odometry(morse.builder.creator.SensorCreator):
  def __init__(self, name="Odometry"):
    morse.builder.creator.SensorCreator.__init__(self, name, 
      "morse/sensors/odometry", "OdometryClass", "morse_odometry")
    mesh = morse.builder.creator.Cube("OdometryCube")
    mesh.scale = (.1,.1,.05)
    self.append(mesh)

class Pose(morse.builder.creator.SensorCreator):
  def __init__(self, name="Pose_sensor"):
    morse.builder.creator.SensorCreator.__init__(self, name, 
      "morse/sensors/pose", "PoseClass", "morse_pose")
    mesh = morse.builder.creator.Cylinder("PoseCylinder")
    mesh.scale = (.1,.1,.2)
    self.append(mesh)

class Proximity(morse.builder.creator.SensorCreator):
  def __init__(self, name="Proximity"):
    morse.builder.creator.SensorCreator.__init__(self, name, 
      "morse/sensors/proximity", "ProximitySensorClass", "morse_proximity")
    mesh = morse.builder.creator.Cylinder("ProximityCylinder")
    mesh.scale = (.1,.1,.2)
    self.append(mesh)
    self.properties(Range = 30.0)


