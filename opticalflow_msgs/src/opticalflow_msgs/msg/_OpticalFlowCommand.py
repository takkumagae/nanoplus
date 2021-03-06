"""autogenerated by genmsg_py from OpticalFlowCommand.msg. Do not edit."""
import roslib.message
import struct


class OpticalFlowCommand(roslib.message.Message):
  _md5sum = "db36d27026ddc82e63c65d723a416964"
  _type = "opticalflow_msgs/OpticalFlowCommand"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64 vx
float64 vy
float64 t
int32 mode

"""
  __slots__ = ['vx','vy','t','mode']
  _slot_types = ['float64','float64','float64','int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       vx,vy,t,mode
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(OpticalFlowCommand, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.vx is None:
        self.vx = 0.
      if self.vy is None:
        self.vy = 0.
      if self.t is None:
        self.t = 0.
      if self.mode is None:
        self.mode = 0
    else:
      self.vx = 0.
      self.vy = 0.
      self.t = 0.
      self.mode = 0

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
      buff.write(_struct_3di.pack(_x.vx, _x.vy, _x.t, _x.mode))
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
      end += 28
      (_x.vx, _x.vy, _x.t, _x.mode,) = _struct_3di.unpack(str[start:end])
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
      buff.write(_struct_3di.pack(_x.vx, _x.vy, _x.t, _x.mode))
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
      end += 28
      (_x.vx, _x.vy, _x.t, _x.mode,) = _struct_3di.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_3di = struct.Struct("<3di")
