"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class act_meas(object):
    __slots__ = ["position", "velocity", "torque"]

    __typenames__ = ["float", "float", "float"]

    __dimensions__ = [None, None, None]

    def __init__(self):
        self.position = 0.0
        self.velocity = 0.0
        self.torque = 0.0

    def encode(self):
        buf = BytesIO()
        buf.write(act_meas._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">fff", self.position, self.velocity, self.torque))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != act_meas._get_packed_fingerprint():
            raise ValueError("Decode error")
        return act_meas._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = act_meas()
        self.position, self.velocity, self.torque = struct.unpack(">fff", buf.read(12))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if act_meas in parents: return 0
        tmphash = (0xd2c2648591a0fc2a) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if act_meas._packed_fingerprint is None:
            act_meas._packed_fingerprint = struct.pack(">Q", act_meas._get_hash_recursive([]))
        return act_meas._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

