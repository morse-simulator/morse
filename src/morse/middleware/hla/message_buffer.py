"""
Provide an implementation of MessageBuffer as used by CERTI
"""

import sys
import hla.omt as fom
from functools import partial

class MessageBufferReader:
    def __init__(self, data):
        self.data = data
        self.index = 0
        self.endian = self.read_octet()
        if self.endian[0] == 0:
            self.read_int16 = partial(self.__read_fom, fom.HLAinteger16LE)
            self.read_int32 = partial(self.__read_fom, fom.HLAinteger32LE)
            self.read_int64 = partial(self.__read_fom, fom.HLAinteger64LE)
            self.read_float = partial(self.__read_fom, fom.HLAfloat32LE)
            self.read_double = partial(self.__read_fom, fom.HLAfloat64LE)
        else:
            self.read_int16 = partial(self.__read_fom, fom.HLAinteger16BE)
            self.read_int32 = partial(self.__read_fom, fom.HLAinteger32BE)
            self.read_int64 = partial(self.__read_fom, fom.HLAinteger64BE)
            self.read_float = partial(self.__read_fom, fom.HLAfloat32BE)
            self.read_double = partial(self.__read_fom, fom.HLAfloat64BE)
        self.size = self.read_int32()

    def __read_fom(self, obj):
        value, off = obj.unpack(self.data[self.index:])
        self.index += off
        return value

    def read_octet(self):
        return self.__read_fom(fom.HLAoctet)

    def read_string(self):
        len = self.read_int32()
        res = ""
        for i in (0, len):
            res[i] = self.read_octet()
        return res

class MessageBufferWriter:
    MAGIC_HEADER_SIZE = 5

    def __init__(self):
        self.data = bytes()
        self.little_endian = (sys.byteorder == 'little')
        if self.little_endian:
            self.add_octet = partial(self.__add_data, fom.HLAoctet)
            self.add_int16 = partial(self.__add_data, fom.HLAinteger16LE)
            self.add_int32 = partial(self.__add_data, fom.HLAinteger32LE)
            self.add_int64 = partial(self.__add_data, fom.HLAinteger64LE)
            self.add_float = partial(self.__add_data, fom.HLAfloat32LE)
            self.add_double = partial(self.__add_data, fom.HLAfloat64LE)
            self.write_octet = partial(self.__write_data, fom.HLAoctet)
            self.write_int16 = partial(self.__write_data, fom.HLAinteger16LE)
            self.write_int32 = partial(self.__write_data, fom.HLAinteger32LE)
            self.write_int64 = partial(self.__write_data, fom.HLAinteger64LE)
            self.write_float = partial(self.__write_data, fom.HLAfloat32LE)
            self.write_double = partial(self.__write_data, fom.HLAfloat64LE)
        else:
            self.add_octet = partial(self.__add_data, fom.HLAoctet)
            self.add_int16 = partial(self.__add_data, fom.HLAinteger16BE)
            self.add_int32 = partial(self.__add_data, fom.HLAinteger32BE)
            self.add_int64 = partial(self.__add_data, fom.HLAinteger64BE)
            self.add_float = partial(self.__add_data, fom.HLAfloat32BE)
            self.add_double = partial(self.__add_data, fom.HLAfloat64BE)
            self.write_octet = partial(self.__write_data, fom.HLAoctet)
            self.write_int16 = partial(self.__write_data, fom.HLAinteger16BE)
            self.write_int32 = partial(self.__write_data, fom.HLAinteger32BE)
            self.write_int64 = partial(self.__write_data, fom.HLAinteger64BE)
            self.write_float = partial(self.__write_data, fom.HLAfloat32BE)
            self.write_double = partial(self.__write_data, fom.HLAfloat64BE)

    def __add_data(self, obj, value):
        self.data += obj.pack(value)

    def __write_data(self, obj, value):
        self.__add_data(obj, value)
        return self.write()

    def add_string(self, value):
        len_s = len(value)
        self.add_int32(len_s)
        for i in range(0, len_s):
            self.add_octet(bytes(value[i], 'utf-8'))

    def write_string(self, value):
        self.add_string(value)
        return self.write()

    def write(self):
        res = bytes()
        size_packet = len(self.data) + self.MAGIC_HEADER_SIZE
        if self.little_endian:
            res = fom.HLAoctet.pack(b'\0')
            res += fom.HLAinteger32LE.pack(size_packet)
        else:
            res = fom.HLAoctet.pack(b'\1')
            res += fom.HLAinteger32BE.pack(size_packet)

        res += self.data
        return res
