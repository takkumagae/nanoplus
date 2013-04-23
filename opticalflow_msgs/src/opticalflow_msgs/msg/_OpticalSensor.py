"""autogenerated by genmsg_py from OpticalSensor.msg. Do not edit."""
import roslib.message
import struct


class OpticalSensor(roslib.message.Message):
  _md5sum = "09c9896124e143c4ae0b4fda90de8eaf"
  _type = "opticalflow_msgs/OpticalSensor"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64 x
float64 y
float64 z
float64 vx
float64 vy
float64 vz
float64 vicon_x
float64 vicon_y
float64 vicon_z
float64 vicon_vx
float64 vicon_vy
float64 vicon_vz
float64 roll
float64 pitch
float64 yaw

"""
  __slots__ = ['x','y','z','vx','vy','vz','vicon_x','vicon_y','vicon_z','vicon_vx','vicon_vy','vicon_vz','roll','pitch','yaw']
  _slot_types = ['float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       x,y,z,vx,vy,vz,vicon_x,vicon_y,vicon_z,vicon_vx,vicon_vy,vicon_vz,roll,pitch,yaw
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(OpticalSensor, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.x is None:
        self.x = 0.
      if self.y is None:
        self.y = 0.
      if self.z is None:
        self.z = 0.
      if self.vx is None:
        self.vx = 0.
      if self.vy is None:
        self.vy = 0.
      if self.vz is None:
        self.vz = 0.
      if self.vicon_x is None:
        self.vicon_x = 0.
      if self.vicon_y is None:
        self.vicon_y = 0.
      if self.vicon_z is None:
        self.vicon_z = 0.
      if self.vicon_vx is None:
        self.vicon_vx = 0.
      if self.vicon_vy is None:
        self.vicon_vy = 0.
      if self.vicon_vz is None:
        self.vicon_vz = 0.
      if self.roll is None:
        self.roll = 0.
      if self.pitch is None:
        self.pitch = 0.
      if self.yaw is None:
        self.yaw = 0.
    else:
      self.x = 0.
      self.y = 0.
      self.z = 0.
      self.vx = 0.
      self.vy = 0.
      self.vz = 0.
      self.vicon_x = 0.
      self.vicon_y = 0.
      self.vicon_z = 0.
      self.vicon_vx = 0.
      self.vicon_vy = 0.
      self.vicon_vz = 0.
      self.roll = 0.
      self.pitch = 0.
      self.yaw = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_15d.pack(_x.x, _x.y, _x.z, _x.vx, _x.vy, _x.vz, _x.vicon_x, _x.vicon_y, _x.vicon_z, _x.vicon_vx, _x.vicon_vy, _x.vicon_vz, _x.roll, _x.pitch, _x.yaw))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      _x = self
      start = end
      end += 120
      (_x.x, _x.y, _x.z, _x.vx, _x.vy, _x.vz, _x.vicon_x, _x.vicon_y, _x.vicon_z, _x.vicon_vx, _x.vicon_vy, _x.vicon_vz, _x.roll, _x.pitch, _x.yaw,) = _struct_15d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_15d.pack(_x.x, _x.y, _x.z, _x.vx, _x.vy, _x.vz, _x.vicon_x, _x.vicon_y, _x.vicon_z, _x.vicon_vx, _x.vicon_vy, _x.vicon_vz, _x.roll, _x.pitch, _x.yaw))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 120
      (_x.x, _x.y, _x.z, _x.vx, _x.vy, _x.vz, _x.vicon_x, _x.vicon_y, _x.vicon_z, _x.vicon_vx, _x.vicon_vy, _x.vicon_vz, _x.roll, _x.pitch, _x.yaw,) = _struct_15d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_15d = struct.Struct("<15d")
