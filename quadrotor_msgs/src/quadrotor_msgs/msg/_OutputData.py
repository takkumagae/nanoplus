"""autogenerated by genmsg_py from OutputData.msg. Do not edit."""
import roslib.message
import struct

import geometry_msgs.msg
import std_msgs.msg

class OutputData(roslib.message.Message):
  _md5sum = "5759ee97fd5c090dcdccf7fc3e50d024"
  _type = "quadrotor_msgs/OutputData"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header
uint16 loop_rate
float64 voltage
geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 angular_velocity
geometry_msgs/Vector3 linear_acceleration
float64 pressure_dheight
float64 pressure_height
geometry_msgs/Vector3 magnetic_field
uint8[8] radio_channel
#uint8[4] motor_rpm
uint8 seq

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

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 

float64 x
float64 y
float64 z
"""
  __slots__ = ['header','loop_rate','voltage','orientation','angular_velocity','linear_acceleration','pressure_dheight','pressure_height','magnetic_field','radio_channel','seq']
  _slot_types = ['Header','uint16','float64','geometry_msgs/Quaternion','geometry_msgs/Vector3','geometry_msgs/Vector3','float64','float64','geometry_msgs/Vector3','uint8[8]','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       header,loop_rate,voltage,orientation,angular_velocity,linear_acceleration,pressure_dheight,pressure_height,magnetic_field,radio_channel,seq
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(OutputData, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg._Header.Header()
      if self.loop_rate is None:
        self.loop_rate = 0
      if self.voltage is None:
        self.voltage = 0.
      if self.orientation is None:
        self.orientation = geometry_msgs.msg.Quaternion()
      if self.angular_velocity is None:
        self.angular_velocity = geometry_msgs.msg.Vector3()
      if self.linear_acceleration is None:
        self.linear_acceleration = geometry_msgs.msg.Vector3()
      if self.pressure_dheight is None:
        self.pressure_dheight = 0.
      if self.pressure_height is None:
        self.pressure_height = 0.
      if self.magnetic_field is None:
        self.magnetic_field = geometry_msgs.msg.Vector3()
      if self.radio_channel is None:
        self.radio_channel = chr(0)*8
      if self.seq is None:
        self.seq = 0
    else:
      self.header = std_msgs.msg._Header.Header()
      self.loop_rate = 0
      self.voltage = 0.
      self.orientation = geometry_msgs.msg.Quaternion()
      self.angular_velocity = geometry_msgs.msg.Vector3()
      self.linear_acceleration = geometry_msgs.msg.Vector3()
      self.pressure_dheight = 0.
      self.pressure_height = 0.
      self.magnetic_field = geometry_msgs.msg.Vector3()
      self.radio_channel = chr(0)*8
      self.seq = 0

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
      buff.write(_struct_H16d.pack(_x.loop_rate, _x.voltage, _x.orientation.x, _x.orientation.y, _x.orientation.z, _x.orientation.w, _x.angular_velocity.x, _x.angular_velocity.y, _x.angular_velocity.z, _x.linear_acceleration.x, _x.linear_acceleration.y, _x.linear_acceleration.z, _x.pressure_dheight, _x.pressure_height, _x.magnetic_field.x, _x.magnetic_field.y, _x.magnetic_field.z))
      _x = self.radio_channel
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_struct_8B.pack(*_x))
      else:
        buff.write(_struct_8s.pack(_x))
      buff.write(_struct_B.pack(self.seq))
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
      if self.orientation is None:
        self.orientation = geometry_msgs.msg.Quaternion()
      if self.angular_velocity is None:
        self.angular_velocity = geometry_msgs.msg.Vector3()
      if self.linear_acceleration is None:
        self.linear_acceleration = geometry_msgs.msg.Vector3()
      if self.magnetic_field is None:
        self.magnetic_field = geometry_msgs.msg.Vector3()
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
      end += 130
      (_x.loop_rate, _x.voltage, _x.orientation.x, _x.orientation.y, _x.orientation.z, _x.orientation.w, _x.angular_velocity.x, _x.angular_velocity.y, _x.angular_velocity.z, _x.linear_acceleration.x, _x.linear_acceleration.y, _x.linear_acceleration.z, _x.pressure_dheight, _x.pressure_height, _x.magnetic_field.x, _x.magnetic_field.y, _x.magnetic_field.z,) = _struct_H16d.unpack(str[start:end])
      start = end
      end += 8
      self.radio_channel = str[start:end]
      start = end
      end += 1
      (self.seq,) = _struct_B.unpack(str[start:end])
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
      buff.write(_struct_H16d.pack(_x.loop_rate, _x.voltage, _x.orientation.x, _x.orientation.y, _x.orientation.z, _x.orientation.w, _x.angular_velocity.x, _x.angular_velocity.y, _x.angular_velocity.z, _x.linear_acceleration.x, _x.linear_acceleration.y, _x.linear_acceleration.z, _x.pressure_dheight, _x.pressure_height, _x.magnetic_field.x, _x.magnetic_field.y, _x.magnetic_field.z))
      _x = self.radio_channel
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_struct_8B.pack(*_x))
      else:
        buff.write(_struct_8s.pack(_x))
      buff.write(_struct_B.pack(self.seq))
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
      if self.orientation is None:
        self.orientation = geometry_msgs.msg.Quaternion()
      if self.angular_velocity is None:
        self.angular_velocity = geometry_msgs.msg.Vector3()
      if self.linear_acceleration is None:
        self.linear_acceleration = geometry_msgs.msg.Vector3()
      if self.magnetic_field is None:
        self.magnetic_field = geometry_msgs.msg.Vector3()
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
      end += 130
      (_x.loop_rate, _x.voltage, _x.orientation.x, _x.orientation.y, _x.orientation.z, _x.orientation.w, _x.angular_velocity.x, _x.angular_velocity.y, _x.angular_velocity.z, _x.linear_acceleration.x, _x.linear_acceleration.y, _x.linear_acceleration.z, _x.pressure_dheight, _x.pressure_height, _x.magnetic_field.x, _x.magnetic_field.y, _x.magnetic_field.z,) = _struct_H16d.unpack(str[start:end])
      start = end
      end += 8
      self.radio_channel = str[start:end]
      start = end
      end += 1
      (self.seq,) = _struct_B.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_8B = struct.Struct("<8B")
_struct_8s = struct.Struct("<8s")
_struct_3I = struct.Struct("<3I")
_struct_B = struct.Struct("<B")
_struct_H16d = struct.Struct("<H16d")
