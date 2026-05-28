from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List
import struct

# ---------------------------------------------------------------------------
# Record type
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class TypeInfo:
    index: int            # EtherCAT object-dictionary type index
    name: str             # ETG long name  (e.g. "INTEGER16")
    base_data_type: str   # IEC / short name  (e.g. "INT")
    bit_size: int         # Canonical bit width
    serialize_fn: str     # Name of the serialization subfunction to call
    deserialize_fn: str   # Name of the deserialization subfunction to call


# ---------------------------------------------------------------------------
# Master table  (keyed by object dictionary index)
# ---------------------------------------------------------------------------

BASE_DATA_TYPES: Dict[int, TypeInfo] = {

    # Boolean / generic word types
    0x0001: TypeInfo(0x0001, "BOOLEAN",        "BOOL",   1,   "serialize_bool",    "deserialize_bool"),
    0x001E: TypeInfo(0x001E, "BYTE",           "BYTE",   8,   "serialize_bitn",   "deserialize_bitn"),
    0x001F: TypeInfo(0x001F, "WORD",           "WORD",   16,  "serialize_bitn",  "deserialize_bitn"),
    0x0020: TypeInfo(0x0020, "DWORD",          "DWORD",  32,  "serialize_bitn",  "deserialize_bitn"),

    # Time types (48-bit, special structure)
    0x000C: TypeInfo(0x000C, "TIME_OF_DAY",    "TIME_OF_DAY",    48, "serialize_time48",  "deserialize_time48"),
    0x000D: TypeInfo(0x000D, "TIME_DIFFERENCE","TIME_DIFFERENCE", 48, "serialize_time48",  "deserialize_time48"),

    # Bit strings BIT1 - BIT16
    0x0030: TypeInfo(0x0030, "BIT1",  "BIT1",   1,  "serialize_bitn", "deserialize_bitn"),
    0x0031: TypeInfo(0x0031, "BIT2",  "BIT2",   2,  "serialize_bitn", "deserialize_bitn"),
    0x0032: TypeInfo(0x0032, "BIT3",  "BIT3",   3,  "serialize_bitn", "deserialize_bitn"),
    0x0033: TypeInfo(0x0033, "BIT4",  "BIT4",   4,  "serialize_bitn", "deserialize_bitn"),
    0x0034: TypeInfo(0x0034, "BIT5",  "BIT5",   5,  "serialize_bitn", "deserialize_bitn"),
    0x0035: TypeInfo(0x0035, "BIT6",  "BIT6",   6,  "serialize_bitn", "deserialize_bitn"),
    0x0036: TypeInfo(0x0036, "BIT7",  "BIT7",   7,  "serialize_bitn", "deserialize_bitn"),
    0x0037: TypeInfo(0x0037, "BIT8",  "BIT8",   8,  "serialize_bitn", "deserialize_bitn"),
    0x0038: TypeInfo(0x0038, "BIT9",  "BIT9",   9,  "serialize_bitn", "deserialize_bitn"),
    0x0039: TypeInfo(0x0039, "BIT10", "BIT10",  10, "serialize_bitn", "deserialize_bitn"),
    0x003A: TypeInfo(0x003A, "BIT11", "BIT11",  11, "serialize_bitn", "deserialize_bitn"),
    0x003B: TypeInfo(0x003B, "BIT12", "BIT12",  12, "serialize_bitn", "deserialize_bitn"),
    0x003C: TypeInfo(0x003C, "BIT13", "BIT13",  13, "serialize_bitn", "deserialize_bitn"),
    0x003D: TypeInfo(0x003D, "BIT14", "BIT14",  14, "serialize_bitn", "deserialize_bitn"),
    0x003E: TypeInfo(0x003E, "BIT15", "BIT15",  15, "serialize_bitn", "deserialize_bitn"),
    0x003F: TypeInfo(0x003F, "BIT16", "BIT16",  16, "serialize_bitn", "deserialize_bitn"),

    # Bit arrays
    0x002D: TypeInfo(0x002D, "BITARR8",  "BITARR8",  8,  "serialize_bitn",  "deserialize_bitn"),
    0x002E: TypeInfo(0x002E, "BITARR16", "BITARR16", 16, "serialize_bitn", "deserialize_bitn"),
    0x002F: TypeInfo(0x002F, "BITARR32", "BITARR32", 32, "serialize_bitn", "deserialize_bitn"),

    # Signed integers
    0x0002: TypeInfo(0x0002, "INTEGER8",  "SINT",  8,  "serialize_int",  "deserialize_int"),
    0x0003: TypeInfo(0x0003, "INTEGER16", "INT",   16, "serialize_int", "deserialize_int"),
    0x0010: TypeInfo(0x0010, "INTEGER24", "INT24", 24, "serialize_int",  "deserialize_int"),
    0x0004: TypeInfo(0x0004, "INTEGER32", "DINT",  32, "serialize_int", "deserialize_int"),
    0x0012: TypeInfo(0x0012, "INTEGER40", "INT40", 40, "serialize_int",  "deserialize_int"),
    0x0013: TypeInfo(0x0013, "INTEGER48", "INT48", 48, "serialize_int",  "deserialize_int"),
    0x0014: TypeInfo(0x0014, "INTEGER56", "INT56", 56, "serialize_int",  "deserialize_int"),
    0x0015: TypeInfo(0x0015, "INTEGER64", "LINT",  64, "serialize_int", "deserialize_int"),

    # Unsigned integers
    0x0005: TypeInfo(0x0005, "UNSIGNED8",  "USINT",  8,  "serialize_uint",  "deserialize_uint"),
    0x0006: TypeInfo(0x0006, "UNSIGNED16", "UINT",   16, "serialize_uint", "deserialize_uint"),
    0x0016: TypeInfo(0x0016, "UNSIGNED24", "UINT24", 24, "serialize_uint",  "deserialize_uint"),
    0x0007: TypeInfo(0x0007, "UNSIGNED32", "UDINT",  32, "serialize_uint", "deserialize_uint"),
    0x0018: TypeInfo(0x0018, "UNSIGNED40", "UINT40", 40, "serialize_uint",  "deserialize_uint"),
    0x0019: TypeInfo(0x0019, "UNSIGNED48", "UINT48", 48, "serialize_uint",  "deserialize_uint"),
    0x001A: TypeInfo(0x001A, "UNSIGNED56", "UINT56", 56, "serialize_uint",  "deserialize_uint"),
    0x001B: TypeInfo(0x001B, "UNSIGNED64", "ULINT",  64, "serialize_uint", "deserialize_uint"),

    # Floating point
    0x0008: TypeInfo(0x0008, "REAL32", "REAL",  32, "serialize_float", "deserialize_float"),
    0x0011: TypeInfo(0x0011, "REAL64", "LREAL", 64, "serialize_float", "deserialize_float"),

    # GUID
    0x001D: TypeInfo(0x001D, "GUID", "GUID", 128, "serialize_guid", "deserialize_guid"),
}


# ---------------------------------------------------------------------------
# Secondary lookup dicts  (built once at import time)
# ---------------------------------------------------------------------------

BY_NAME: Dict[str, TypeInfo] = {
    info.name: info for info in BASE_DATA_TYPES.values()
}

BY_BASE_DATA_TYPE: Dict[str, TypeInfo] = {
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
    """
    Retrieve a TypeInfo record by any one of the three keys.

    Examples
    --------
    get_type_info(index=0x0003)
    get_type_info(name="INTEGER16")
    get_type_info(base_data_type="INT")
    """
    if index is not None:
        try:
            return BASE_DATA_TYPES[index]
        except KeyError:
            raise KeyError(f"No EtherCAT type with index {index:#06x}") from None

    if name is not None:
        try:
            return BY_NAME[name]
        except KeyError:
            raise KeyError(f"No EtherCAT type with name '{name}'") from None

    if base_data_type is not None:
        try:
            return BY_BASE_DATA_TYPE[base_data_type]
        except KeyError:
            raise KeyError(f"No EtherCAT type with base_data_type '{base_data_type}'") from None

    raise ValueError("Provide at least one of: index, name, base_data_type")


# ---------------------------------------------------------------------------
# Generic functions
# ---------------------------------------------------------------------------

def serialize(
    value,
    *,
    index: int | None = None,
    name: str | None = None,
    base_data_type: str | None = None,
) -> List[int]:
    object_info = get_type_info(index=index, name=name, base_data_type=base_data_type)
    return globals()[object_info.serialize_fn](value, object_info.bit_size)

def deserialize(
    serialized_value: List[int],
    *,
    index: int | None = None,
    name: str | None = None,
    base_data_type: str | None = None,
):
    if not isinstance(serialized_value, list) or not all(isinstance(b, int) for b in serialized_value):
        raise TypeError("serialized_value must be a list[int]")
    object_info = get_type_info(index=index, name=name, base_data_type=base_data_type)
    if (object_info.bit_size + 7) // 8 != len(serialized_value):
        raise ValueError(
            "number of bytes needed to contain object_info.bit_size must be equal to serialized_value length")
    return globals()[object_info.deserialize_fn](serialized_value, object_info.bit_size)


# ---------------------------------------------------------------------------
# Specific functions (called by generic functions)
# ---------------------------------------------------------------------------

def serialize_bitn(val, bit_s: int) -> List[int]:
    """
    Serialize a bit-string into a list[int].
    """
    if type(val) is str:
        vali = int(val, 2)
    elif type(val) is int:
        vali = val
    else:
        raise TypeError(f"val must be either int or str, not {type(val)}")

    byte_len = (bit_s + 7) // 8
    return list(vali.to_bytes(byte_len, byteorder="little"))

def deserialize_bitn(ser_val: List[int], bit_s: int) -> str:
    """
    Deserialize a list[int] into a bit-string.
    """
    value = int.from_bytes(bytes(ser_val), byteorder='little')
    return bin(value)[2:].zfill(bit_s)

def serialize_int(val: int, bit_s: int) -> List[int]:
    """
    Serialize a signed int into a list[int].
    """
    byte_len = (bit_s + 7) // 8
    return list(val.to_bytes(byte_len, byteorder="little", signed=True))

def deserialize_int(ser_val: List[int], bit_s: int) -> int:
    """
    Deserialize a list[int] into a signed int.
    """
    return int.from_bytes(bytes(ser_val), byteorder='little', signed=True)

def serialize_uint(val: int, bit_s: int) -> List[int]:
    """
    Serialize an unsigned int into a list[int].
    """
    byte_len = (bit_s + 7) // 8
    return list(val.to_bytes(byte_len, byteorder="little"))

def deserialize_uint(ser_val: List[int], bit_s: int) -> int:
    """
    Deserialize a list[int] into an unsigned int.
    """
    return int.from_bytes(bytes(ser_val), byteorder='little')

def serialize_bool(val, bit_s: int) -> List[int]:
    """
    Serialize a bool into a list[int].
    """
    if type(val) is bool:
        val = int(val)
    return serialize_bitn(val, bit_s)

def deserialize_bool(ser_val: List[int], bit_s: int) -> bool:
    """
    Deserialize a list[int] into a bool.
    """
    return bool(int.from_bytes(bytes(ser_val), byteorder='little'))

def serialize_float(val, bit_s: int) -> List[int]:
    """
    Serialize a float into a list[int].
    """
    return list(struct.pack(f"<{'f' if bit_s == 32 else 'd'}", val))

def deserialize_float(ser_val: List[int], bit_s: int) -> float:
    """
    Deserialize a list[int] into a float.
    """
    return struct.unpack(f"<{'f' if bit_s == 32 else 'd'}", bytes(ser_val))[0]