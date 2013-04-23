"""autogenerated by genmsg_py from OpticalFlow.msg. Do not edit."""
import roslib.message
import struct

import std_msgs.msg

class OpticalFlow(roslib.message.Message):
  _md5sum = "6705fe0e94fea14978a508d00cf97427"
  _type = "px_comm/OpticalFlow"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header

float32 ground_distance  # distance to ground in meters
int16   flow_x           # x-component of optical flow in pixels
int16   flow_y           # y-component of optical flow in pixels
float32 velocity_x       # x-component of scaled optical flow in m/s
float32 velocity_y       # y-component of scaled optical flow in m/s
uint8   quality          # quality of optical flow estimate

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

"""
  __slots__ = ['header','ground_distance','flow_x','flow_y','velocity_x','velocity_y','quality']
  _slot_types = ['Header','float32','int16','int16','float32','float32','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       header,ground_distance,flow_x,flow_y,velocity_x,velocity_y,quality
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(OpticalFlow, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg._Header.Header()
      if self.ground_distance is None:
        self.ground_distance = 0.
      if self.flow_x is None:
        self.flow_x = 0
      if self.flow_y is None:
        self.flow_y = 0
      if self.velocity_x is None:
        self.velocity_x = 0.
      if self.velocity_y is None:
        self.velocity_y = 0.
      if self.quality is None:
        self.quality = 0
    else:
      self.header = std_msgs.msg._Header.Header()
      self.ground_distance = 0.
      self.flow_x = 0
      self.flow_y = 0
      self.velocity_x = 0.
      self.velocity_y = 0.
      self.quality = 0

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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_f2h2fB.pack(_x.ground_distance, _x.flow_x, _x.flow_y, _x.velocity_x, _x.velocity_y, _x.quality))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg._Header.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 17
      (_x.ground_distance, _x.flow_x, _x.flow_y, _x.velocity_x, _x.velocity_y, _x.quality,) = _struct_f2h2fB.unpack(str[start:end])
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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_f2h2fB.pack(_x.ground_distance, _x.flow_x, _x.flow_y, _x.velocity_x, _x.velocity_y, _x.quality))
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
      if self.header is None:
        self.header = std_msgs.msg._Header.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 17
      (_x.ground_distance, _x.flow_x, _x.flow_y, _x.velocity_x, _x.velocity_y, _x.quality,) = _struct_f2h2fB.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_3I = struct.Struct("<3I")
_struct_f2h2fB = struct.Struct("<f2h2fB")
