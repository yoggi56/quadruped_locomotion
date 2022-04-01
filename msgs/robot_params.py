"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class robot_params(object):
    __slots__ = ["robot_dx", "robot_dy", "robot_dz", "robot_step_freq", "robot_step_lenght", "robot_step_height", "robot_gait_type"]

    __typenames__ = ["float", "float", "float", "float", "float", "float", "int16_t"]

    __dimensions__ = [None, None, None, None, None, None, None]

    def __init__(self):
        self.robot_dx = 0.0
        self.robot_dy = 0.0
        self.robot_dz = 0.0
        self.robot_step_freq = 0.0
        self.robot_step_lenght = 0.0
        self.robot_step_height = 0.0
        self.robot_gait_type = 0

    def encode(self):
        buf = BytesIO()
        buf.write(robot_params._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">ffffffh", self.robot_dx, self.robot_dy, self.robot_dz, self.robot_step_freq, self.robot_step_lenght, self.robot_step_height, self.robot_gait_type))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != robot_params._get_packed_fingerprint():
            raise ValueError("Decode error")
        return robot_params._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = robot_params()
        self.robot_dx, self.robot_dy, self.robot_dz, self.robot_step_freq, self.robot_step_lenght, self.robot_step_height, self.robot_gait_type = struct.unpack(">ffffffh", buf.read(26))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if robot_params in parents: return 0
        tmphash = (0xb91e080e9cc605fa) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if robot_params._packed_fingerprint is None:
            robot_params._packed_fingerprint = struct.pack(">Q", robot_params._get_hash_recursive([]))
        return robot_params._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

