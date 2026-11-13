"""Functions to serialize/deserialize a value according to EtherCAT data type.

The data type can be identified by its index, name, or base data type
string. Providing one is sufficient.

For definitions of supported base data types and encoding see see ETG.1000.6
and ETG.1020 at https://www.ethercat.org/en/downloads.html.
"""
import struct
from dataclasses import dataclass

# ---------------------------------------------------------------------------
# Record type
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TypeInfo:
    """Stores attributes of EtherCAT base data types from ETG.1020.

    For definitions of supported base data types and encoding
    see see ETG.1000.6, ETG.1020 at https://www.ethercat.org/en/downloads.html.
    """

    index: int            # EtherCAT datatype index
    name: str             # ETG long name  (e.g. 'INTEGER16')
    base_data_type: str   # IEC / short name  (e.g. 'INT')
    bit_size: int         # Canonical bit width
    serialize_fn: str     # Name of the serialization subfunction to call
    deserialize_fn: str   # Name of the deserialization subfunction to call


# ---------------------------------------------------------------------------
# Master table
# ---------------------------------------------------------------------------

BASE_DATA_TYPES: dict[int, TypeInfo] = {

    # Boolean / generic word types
    0x0001: TypeInfo(0x0001, 'BOOLEAN', 'BOOL',  1,  'serialize_bool', 'deserialize_bool'),
    0x001E: TypeInfo(0x001E, 'BYTE',    'BYTE',  8,  'serialize_bitn', 'deserialize_bitn'),
    0x001F: TypeInfo(0x001F, 'WORD',    'WORD',  16, 'serialize_bitn', 'deserialize_bitn'),
    0x0020: TypeInfo(0x0020, 'DWORD',   'DWORD', 32, 'serialize_bitn', 'deserialize_bitn'),

    # Time types (48-bit, special structure)
    0x000C: TypeInfo(0x000C, 'TIME_OF_DAY',     'TIME_OF_DAY',     48, 'serialize_time_of_day',     'deserialize_time48'),
    0x000D: TypeInfo(0x000D, 'TIME_DIFFERENCE', 'TIME_DIFFERENCE', 48, 'serialize_time_difference', 'deserialize_time48'),

    # Bit strings BIT1 - BIT16
    0x0030: TypeInfo(0x0030, 'BIT1',  'BIT1',  1,  'serialize_bitn', 'deserialize_bitn'),
    0x0031: TypeInfo(0x0031, 'BIT2',  'BIT2',  2,  'serialize_bitn', 'deserialize_bitn'),
    0x0032: TypeInfo(0x0032, 'BIT3',  'BIT3',  3,  'serialize_bitn', 'deserialize_bitn'),
    0x0033: TypeInfo(0x0033, 'BIT4',  'BIT4',  4,  'serialize_bitn', 'deserialize_bitn'),
    0x0034: TypeInfo(0x0034, 'BIT5',  'BIT5',  5,  'serialize_bitn', 'deserialize_bitn'),
    0x0035: TypeInfo(0x0035, 'BIT6',  'BIT6',  6,  'serialize_bitn', 'deserialize_bitn'),
    0x0036: TypeInfo(0x0036, 'BIT7',  'BIT7',  7,  'serialize_bitn', 'deserialize_bitn'),
    0x0037: TypeInfo(0x0037, 'BIT8',  'BIT8',  8,  'serialize_bitn', 'deserialize_bitn'),
    0x0038: TypeInfo(0x0038, 'BIT9',  'BIT9',  9,  'serialize_bitn', 'deserialize_bitn'),
    0x0039: TypeInfo(0x0039, 'BIT10', 'BIT10', 10, 'serialize_bitn', 'deserialize_bitn'),
    0x003A: TypeInfo(0x003A, 'BIT11', 'BIT11', 11, 'serialize_bitn', 'deserialize_bitn'),
    0x003B: TypeInfo(0x003B, 'BIT12', 'BIT12', 12, 'serialize_bitn', 'deserialize_bitn'),
    0x003C: TypeInfo(0x003C, 'BIT13', 'BIT13', 13, 'serialize_bitn', 'deserialize_bitn'),
    0x003D: TypeInfo(0x003D, 'BIT14', 'BIT14', 14, 'serialize_bitn', 'deserialize_bitn'),
    0x003E: TypeInfo(0x003E, 'BIT15', 'BIT15', 15, 'serialize_bitn', 'deserialize_bitn'),
    0x003F: TypeInfo(0x003F, 'BIT16', 'BIT16', 16, 'serialize_bitn', 'deserialize_bitn'),

    # Bit arrays
    0x002D: TypeInfo(0x002D, 'BITARR8',  'BITARR8',  8,  'serialize_bitn', 'deserialize_bitn'),
    0x002E: TypeInfo(0x002E, 'BITARR16', 'BITARR16', 16, 'serialize_bitn', 'deserialize_bitn'),
    0x002F: TypeInfo(0x002F, 'BITARR32', 'BITARR32', 32, 'serialize_bitn', 'deserialize_bitn'),

    # Signed integers
    0x0002: TypeInfo(0x0002, 'INTEGER8',  'SINT',  8,  'serialize_int', 'deserialize_int'),
    0x0003: TypeInfo(0x0003, 'INTEGER16', 'INT',   16, 'serialize_int', 'deserialize_int'),
    0x0010: TypeInfo(0x0010, 'INTEGER24', 'INT24', 24, 'serialize_int', 'deserialize_int'),
    0x0004: TypeInfo(0x0004, 'INTEGER32', 'DINT',  32, 'serialize_int', 'deserialize_int'),
    0x0012: TypeInfo(0x0012, 'INTEGER40', 'INT40', 40, 'serialize_int', 'deserialize_int'),
    0x0013: TypeInfo(0x0013, 'INTEGER48', 'INT48', 48, 'serialize_int', 'deserialize_int'),
    0x0014: TypeInfo(0x0014, 'INTEGER56', 'INT56', 56, 'serialize_int', 'deserialize_int'),
    0x0015: TypeInfo(0x0015, 'INTEGER64', 'LINT',  64, 'serialize_int', 'deserialize_int'),

    # Unsigned integers
    0x0005: TypeInfo(0x0005, 'UNSIGNED8',  'USINT',  8,  'serialize_uint', 'deserialize_uint'),
    0x0006: TypeInfo(0x0006, 'UNSIGNED16', 'UINT',   16, 'serialize_uint', 'deserialize_uint'),
    0x0016: TypeInfo(0x0016, 'UNSIGNED24', 'UINT24', 24, 'serialize_uint', 'deserialize_uint'),
    0x0007: TypeInfo(0x0007, 'UNSIGNED32', 'UDINT',  32, 'serialize_uint', 'deserialize_uint'),
    0x0018: TypeInfo(0x0018, 'UNSIGNED40', 'UINT40', 40, 'serialize_uint', 'deserialize_uint'),
    0x0019: TypeInfo(0x0019, 'UNSIGNED48', 'UINT48', 48, 'serialize_uint', 'deserialize_uint'),
    0x001A: TypeInfo(0x001A, 'UNSIGNED56', 'UINT56', 56, 'serialize_uint', 'deserialize_uint'),
    0x001B: TypeInfo(0x001B, 'UNSIGNED64', 'ULINT',  64, 'serialize_uint', 'deserialize_uint'),

    # Floating point
    0x0008: TypeInfo(0x0008, 'REAL32', 'REAL',  32, 'serialize_float', 'deserialize_float'),
    0x0011: TypeInfo(0x0011, 'REAL64', 'LREAL', 64, 'serialize_float', 'deserialize_float'),

    # GUID (according to specifications value is stored as a 128-bit integer)
    0x001D: TypeInfo(0x001D, 'GUID', 'GUID', 128, 'serialize_int', 'deserialize_int'),

    # - Base Data Types with variable length -
    # Strings
    0x0009: TypeInfo(0x0009, 'VISIBLE_STRING', 'STRING(n)',  8,  'serialize_visible_string', 'deserialize_visible_string'),  # 8*(n)),
    0x0268: TypeInfo(0x0268, 'UNICODE_STRING', 'WSTRING(n)', 16, 'serialize_unicode_string', 'deserialize_unicode_string'),  # 16*(n)

    # Octet field
    0x000A: TypeInfo(0x000A, 'OCTET_STRING',      'ARRAY [0..n] OF BYTE',     8,  'serialize_bitn',  'deserialize_bitn'),  # 8*(n+1)
    0x000B: TypeInfo(0x000B, 'ARRAY_OF_UINT',     'ARRAY [0..n] OF UINT',     16, 'serialize_uint',  'deserialize_uint'),  # 16*(n+1)
    0x0260: TypeInfo(0x0260, 'ARRAY_OF_INT',      'ARRAY [0..n] OF INT',      16, 'serialize_int',   'deserialize_int'),  # 16*(n+1)
    0x0261: TypeInfo(0x0261, 'ARRAY_OF_SINT',     'ARRAY [0..n] OF SINT',     8,  'serialize_int',   'deserialize_int'),  # 8*(n+1)
    0x0262: TypeInfo(0x0262, 'ARRAY_OF_DINT',     'ARRAY [0..n] OF DINT',     32, 'serialize_int',   'deserialize_int'),  # 32*(n+1)
    0x0263: TypeInfo(0x0263, 'ARRAY_OF_UDINT',    'ARRAY [0..n] OF UDINT',    32, 'serialize_uint',  'deserialize_uint'),  # 32*(n+1)
    0x0264: TypeInfo(0x0264, 'ARRAY_OF_BITARR8',  'ARRAY [0..n] OF BITARR8',  8,  'serialize_bitn',  'deserialize_bitn'),  # 8*(n+1)
    0x0265: TypeInfo(0x0265, 'ARRAY_OF_BITARR16', 'ARRAY [0..n] OF BITARR16', 16, 'serialize_bitn',  'deserialize_bitn'),  # 16*(n+1)
    0x0266: TypeInfo(0x0266, 'ARRAY_OF_BITARR32', 'ARRAY [0..n] OF BITARR32', 32, 'serialize_bitn',  'deserialize_bitn'),  # 32*(n+1)
    0x0267: TypeInfo(0x0267, 'ARRAY_OF_USINT',    'ARRAY [0..n] OF USINT',    8,  'serialize_uint',  'deserialize_uint'),  # 8*(n+1)
    0x0269: TypeInfo(0x0269, 'ARRAY_OF_REAL',     'ARRAY [0..n] OF REAL',     32, 'serialize_float', 'deserialize_float'),  # 32*(n+1)
    0x026A: TypeInfo(0x026A, 'ARRAY_OF_LREAL',    'ARRAY [0..n] OF LREAL',    64, 'serialize_float', 'deserialize_float'),  # 64*(n+1)
}


# ---------------------------------------------------------------------------
# Secondary lookup dicts  (built once at import time)
# ---------------------------------------------------------------------------

BY_NAME: dict[str, TypeInfo] = {
    info.name: info for info in BASE_DATA_TYPES.values()
}

BY_BASE_DATA_TYPE: dict[str, TypeInfo] = {
    info.base_data_type: info for info in BASE_DATA_TYPES.values()
}


# ---------------------------------------------------------------------------
# Convenience accessor
# ---------------------------------------------------------------------------

def get_type_info(
    *,
    index: int | None = None,
    name: str | None = None,
    base_data_type: str | None = None,
) -> TypeInfo:
    """Retrieve a TypeInfo record by any one of the three keys.

    Examples
    --------
    get_type_info(index=0x0003)
    get_type_info(name='INTEGER16')
    get_type_info(base_data_type='INT')
    """
    if index is not None:
        try:
            return BASE_DATA_TYPES[index]
        except KeyError:
            raise KeyError(
                f'No EtherCAT type with index {index:#04x}'
            ) from None

    if name is not None:
        try:
            return BY_NAME[name]
        except KeyError:
            raise KeyError(
                f"No EtherCAT type with name '{name}'"
            ) from None

    if base_data_type is not None:
        try:
            return BY_BASE_DATA_TYPE[base_data_type]
        except KeyError:
            raise KeyError(
                f"No EtherCAT type with base_data_type '{base_data_type}'"
            ) from None

    raise ValueError('Provide at least one of: index, name, base_data_type')


# ---------------------------------------------------------------------------
# Generic functions
# ---------------------------------------------------------------------------

def serialize(
    value,
    *,
    index: int | None = None,
    name: str | None = None,
    base_data_type: str | None = None,
) -> list[int]:
    """Serialize a value according to an EtherCAT base data type.

    The data type can be identified by its index, name, or base data type
    string. Providing one is sufficient.

    For definitions of supported base data types and encoding
    see see ETG.1000.6 and ETG.1020 at https://www.ethercat.org/en/downloads.html

    :param value:
        Value to serialize.
    :param index:
        EtherCAT data type index.
    :param name:
        EtherCAT data type name.
    :param base_data_type:
        Data type making up this base data type. Called 'Type' in
        Somanet Circulo Object Dictionary reference.
    :returns:
        Serialized value as a list of bytes.
    :rtype:
        list[int]
    """
    object_info = get_type_info(index=index, name=name, base_data_type=base_data_type)

    # serialize a base data type
    if object_info.base_data_type[0:5] != 'ARRAY':
        return globals()[object_info.serialize_fn](
            value, object_info.bit_size)
    # serialize a base data type list (imagine this: base_data_type[])
    else:
        out = []
        for item in value:
            out.extend(globals()[object_info.serialize_fn](
                item, object_info.bit_size))
        return out


def deserialize(
    serialized_value: list[int],
    *,
    index: int | None = None,
    name: str | None = None,
    base_data_type: str | None = None,
):
    """Deserializes a list of bytes representing an EtherCAT base data type.

    The data type can be identified by its index, name, or base data type
    string. Providing one is sufficient.

    For definitions of supported base data types and encoding
    see ETG.1000.6 and ETG.1020 at https://www.ethercat.org/en/downloads.html

    :param serialized_value:
        Bytes to deserialize.
    :param index:
        EtherCAT data type index.
    :param name:
        EtherCAT data type name.
    :param base_data_type:
        Data type making up this base data type. Called 'Type' in
        Somanet Circulo Object Dictionary reference.
    :returns:
        Deserialized value as a fitting python data type.
    """
    if not isinstance(serialized_value, list) or not all(isinstance(b, int) for b in serialized_value):
        raise TypeError(f'serialized_value must be a list[int] not {type(serialized_value)} {serialized_value}')
    object_info = get_type_info(index=index, name=name, base_data_type=base_data_type)
    byte_len = (object_info.bit_size + 7) // 8
    isArray = object_info.base_data_type[0:5] == 'ARRAY'

    # check if size of variably sized serialized object is plausible
    if object_info.name[-6:] == 'STRING' or isArray:
        if len(serialized_value) % byte_len != 0:
            raise ValueError(
                'Number of bytes needed to contain variably sized list of Base Data Types must '
                'be evenly divisible by the byte-size of those Base Data Types.')
    # check if size of serialized object is correct
    else:
        if byte_len != len(serialized_value):
            raise ValueError(
                'Number of bytes needed to contain object_info.bit_size must be equal to '
                'serialized_value length.')

    # deserialize a serialized base data type
    if not isArray:
        return globals()[object_info.deserialize_fn](serialized_value, object_info.bit_size)
    # deserialize a serialized base data type array (imagine this: base_data_type[])
    else:
        out = []
        for i in range(0, len(serialized_value), byte_len):
            out.append(globals()[object_info.deserialize_fn](serialized_value[i:i + byte_len],
                                                                object_info.bit_size))
        return out

# ---------------------------------------------------------------------------
# Specific functions (called by generic functions)
# ---------------------------------------------------------------------------


def serialize_bitn(val, bit_s: int) -> list[int]:
    """Serialize a bit-string into a list[int]."""
    if isinstance(val, str):
        val = int(val, 2)
    elif not isinstance(val, int):
        raise TypeError(f'Input value must be either int or str, not {type(val)} {val}.')

    byte_len = (bit_s + 7) // 8
    return list(val.to_bytes(byte_len, byteorder='little'))


def deserialize_bitn(ser_val: list[int], bit_s: int) -> str:
    """Deserialize a list[int] into a bit-string."""
    value = int.from_bytes(bytes(ser_val), byteorder='little')
    return bin(value)[2:].zfill(bit_s)


def serialize_int(val: int, bit_s: int) -> list[int]:
    """Serialize a signed int into a list[int]."""
    if not isinstance(val, int):
        raise TypeError(f'Input value must be an int, not {type(val)} {val}.')
    byte_len = (bit_s + 7) // 8
    return list(val.to_bytes(byte_len, byteorder='little', signed=True))


def deserialize_int(ser_val: list[int], bit_s: int) -> int:
    """Deserialize a list[int] into a signed int."""
    return int.from_bytes(bytes(ser_val), byteorder='little', signed=True)


def serialize_uint(val: int, bit_s: int) -> list[int]:
    """Serialize an unsigned int into a list[int]."""
    if not isinstance(val, int):
        raise TypeError(f'Input value must be an int, not {type(val)} {val}.')
    byte_len = (bit_s + 7) // 8
    return list(val.to_bytes(byte_len, byteorder='little'))


def deserialize_uint(ser_val: list[int], bit_s: int) -> int:
    """Deserialize a list[int] into an unsigned int."""
    return int.from_bytes(bytes(ser_val), byteorder='little')


def serialize_bool(val, bit_s: int) -> list[int]:
    """Serialize a bool into a list[int]."""
    if (not isinstance(val, bool)) and (val not in [1, 0]):
        raise TypeError(f'Input value must be a bool or the int 1 or 0, not {type(val)} {val}.')
    if val:
        return serialize_bitn(0xff, bit_s)
    else:
        return serialize_bitn(0x00, bit_s)


def deserialize_bool(ser_val: list[int], bit_s: int) -> bool:
    """Deserialize a list[int] into a bool."""
    return 0 != ser_val[0]


def serialize_float(val, bit_s: int) -> list[int]:
    """Serialize a float into a list[int]."""
    if not isinstance(val, float):
        raise TypeError(f'Input value must be a float, not {type(val)} {val}.')
    return list(struct.pack(f"<{'f' if bit_s == 32 else 'd'}", val))


def deserialize_float(ser_val: list[int], bit_s: int) -> float:
    """Deserialize a list[int] into a float."""
    return struct.unpack(f"<{'f' if bit_s == 32 else 'd'}", bytes(ser_val))[0]


def serialize_time_of_day(val, bit_s: int) -> list[int]:
    """Serialize tuple of ints (ms since midnight, days since 01.01.1984)."""
    if (len(val) != 2) or (not isinstance(val[0], int)) or (not isinstance(val[1], int)):
        raise TypeError(f'There needs to be a ms and a day value. '
                        f'They must be ints in a tuple, not {type(val)} {val}')
    if val[0] > (1 << 28) - 1:
        raise OverflowError(
            'the 4 most significant bits of number of ms since midnight need to be 0')
    return list(val[0].to_bytes(4, byteorder='big')) + list(val[1].to_bytes(2, byteorder='big'))


def serialize_time_difference(val, bit_s: int) -> list[int]:
    """Serialize tuple of ints (ms, days)."""
    if (len(val) != 2) or (not isinstance(val[0], int)) or (not isinstance(val[1], int)):
        raise TypeError('There needs to be a ms and a day value. '
                        f'They must be ints in a tuple, not {type(val)} {val}')
    return list(val[0].to_bytes(4, byteorder='big')) + list(val[1].to_bytes(2, byteorder='big'))


def deserialize_time48(ser_val: list[int], bit_s: int) -> tuple[int]:
    """Deserialize list[int] to tuple of ints (ms, days)."""
    return (int.from_bytes(ser_val[:4], byteorder='big'),
            int.from_bytes(ser_val[4:], byteorder='big'))


def serialize_visible_string(val: str, bit_s: int) -> list[int]:
    """Serialize an UTF-8 encoded string into a list[int]."""
    if not isinstance(val, str):
        raise TypeError(f'Input value must be str, not {type(val)} {val}')
    return list(val.encode('UTF-8'))


def deserialize_visible_string(ser_val: list[int], bit_s: int) -> str:
    """Deserialize a list[int] into an UTF-8 encoded string."""
    return bytes(ser_val).decode('UTF-8')


def serialize_unicode_string(val: str, bit_s: int) -> list[int]:
    """Serialize a utf_16_le encoded string into a list[int]."""
    if not isinstance(val, str):
        raise TypeError(f'Input value must be str, not {type(val)} {val}')
    return list(val.encode('utf_16_le'))


def deserialize_unicode_string(ser_val: list[int], bit_s: int) -> str:
    """Deerialize a list[int] into a utf_16_le encoded string."""
    return bytes(ser_val).decode('utf_16_le')
