"""Tests for sdo_serializer.py module."""
import random
import pytest
from rise_motion.sdo_serializer import serialize, deserialize
from math import isnan
from typing import List

# ---------------------------------------------------------------------------
# Bit strings BIT1 - BIT16 tests
# ---------------------------------------------------------------------------

# --- round-trip tests ---

@pytest.mark.parametrize("bit_width", range(1, 17))
def test_roundtrip_random_bitstrings(bit_width: int):
    """Serializing then deserializing a random valid value returns the original."""
    max_val = (1 << bit_width) - 1
    value = format(random.randint(0, max_val), f"0{bit_width}b")
    dtype = f"BIT{bit_width}"
    assert value == deserialize(serialize(value, base_data_type=dtype), base_data_type=dtype)


@pytest.mark.parametrize("bit_width", range(1, 17))
def test_roundtrip_min_bitstrings(bit_width: int):
    """Zero survives a round-trip for every bit width."""
    value = "0".zfill(bit_width)
    dtype = f"BIT{bit_width}"
    assert value == deserialize(serialize(value, base_data_type=dtype), base_data_type=dtype)


@pytest.mark.parametrize("bit_width", range(1, 17))
def test_roundtrip_max_bitstrings(bit_width: int):
    """All-ones value survives a round-trip for every bit width."""
    value = "1" * bit_width
    dtype = f"BIT{bit_width}"
    assert value == deserialize(serialize(value, base_data_type=dtype), base_data_type=dtype)


# --- Explicit serialization checks (known input -> known list[int]) ---

@pytest.mark.parametrize("value, dtype, expected", [
    ("0",                "BIT1",  [0x00]),
    ("1",                "BIT1",  [0x01]),
    ("11",               "BIT2",  [0x03]),
    ("10",               "BIT2",  [0x02]),
    ("1111111111111111", "BIT16", [0xff, 0xff]),
    ("0000000000000000", "BIT16", [0x00, 0x00]),
    ("1000000000000000", "BIT16", [0x00, 0x80]),  # MSB only
])
def test_serialize_known_values_bitstrings(value: str, dtype: str, expected: List[int]):
    assert expected == serialize(value, base_data_type=dtype)


# --- Explicit deserialization checks (known list[int] -> known output) ---

@pytest.mark.parametrize("serialized, dtype, expected_value", [
    ([0x00], "BIT1",  "0"),
    ([0x01], "BIT1",  "1"),
    ([0x03], "BIT2",  "11"),
    ([0x02], "BIT2",  "10"),
    ([0xff, 0xff], "BIT16", "1111111111111111"),
    ([0x00, 0x00], "BIT16", "0000000000000000"),
])
def test_deserialize_known_values_bitstrings(serialized: List[int], dtype: str, expected_value: str):
    assert expected_value == deserialize(serialized, base_data_type=dtype)


# ---------------------------------------------------------------------------
# BITARR8, BITARR16, BITARR32, BYTE, WORD, DWORD tests
# ---------------------------------------------------------------------------

# --- round-trip tests ---

@pytest.mark.parametrize("bit_width, dtype", [
    (8, "BITARR8"),(16, "BITARR16"),(32, "BITARR32"),
    (8, "BYTE"),(16, "WORD"),(32, "DWORD")])
def test_roundtrip_random_bitarr_word(bit_width: int, dtype: str):
    """Serializing then deserializing a random valid value returns the original."""
    max_val = (1 << bit_width) - 1
    value = format(random.randint(0, max_val), f"0{bit_width}b")
    assert value == deserialize(serialize(value, base_data_type=dtype), base_data_type=dtype)


@pytest.mark.parametrize("bit_width, dtype", [
    (8, "BITARR8"),(16, "BITARR16"),(32, "BITARR32"),
    (8, "BYTE"),(16, "WORD"),(32, "DWORD")])
def test_roundtrip_min_bitarr_word(bit_width: int, dtype: str):
    """Zero survives a round-trip for every bit width."""
    value = "0".zfill(bit_width)
    assert value == deserialize(serialize(value, base_data_type=dtype), base_data_type=dtype)


@pytest.mark.parametrize("bit_width, dtype", [
    (8, "BITARR8"),(16, "BITARR16"),(32, "BITARR32"),
    (8, "BYTE"),(16, "WORD"),(32, "DWORD")])
def test_roundtrip_max_bitarr_word(bit_width: int, dtype: str):
    """All-ones value survives a round-trip for every bit width."""
    value = "1" * bit_width
    assert value == deserialize(serialize(value, base_data_type=dtype), base_data_type=dtype)


# --- Explicit serialization checks (known input -> known list[int]) ---

@pytest.mark.parametrize("value, dtype, expected", [
    ("10101100",                         "BITARR8",  [0xac]),
    ("01010101",                         "BITARR8",  [0x55]),
    ("1100101001110001",                 "BITARR16", [0x71, 0xca]),
    ("0011110000101110",                 "BITARR16", [0x2e, 0x3c]),
    ("10010000111100001010101001101101", "BITARR32", [0x6d, 0xaa, 0xf0, 0x90]),
    ("01101111000101011000001111110000", "BITARR32", [0xf0, 0x83, 0x15, 0x6f]),
    ("10101100",                         "BYTE",     [0xac]),
    ("01010101",                         "BYTE",     [0x55]),
    ("1100101001110001",                 "WORD",     [0x71, 0xca]),
    ("0011110000101110",                 "WORD",     [0x2e, 0x3c]),
    ("10010000111100001010101001101101", "DWORD",    [0x6d, 0xaa, 0xf0, 0x90]),
    ("01101111000101011000001111110000", "DWORD",    [0xf0, 0x83, 0x15, 0x6f]),
])
def test_serialize_known_values_bitarr_word(value: str, dtype: str, expected: List[int]):
    assert expected == serialize(value, base_data_type=dtype)


# --- Explicit deserialization checks (known list[int] -> known output) ---

@pytest.mark.parametrize("serialized, dtype, expected_value", [
    ([0xac],                             "BITARR8",  "10101100"),
    ([0x55],                             "BITARR8",  "01010101"),
    ([0x71, 0xca],                       "BITARR16", "1100101001110001"),
    ([0x2e, 0x3c],                       "BITARR16", "0011110000101110"),
    ([0x6d, 0xaa, 0xf0, 0x90],          "BITARR32", "10010000111100001010101001101101"),
    ([0xf0, 0x83, 0x15, 0x6f],          "BITARR32", "01101111000101011000001111110000"),
    ([0xac],                             "BYTE",     "10101100"),
    ([0x55],                             "BYTE",     "01010101"),
    ([0x71, 0xca],                       "WORD",     "1100101001110001"),
    ([0x2e, 0x3c],                       "WORD",     "0011110000101110"),
    ([0x6d, 0xaa, 0xf0, 0x90],          "DWORD",    "10010000111100001010101001101101"),
    ([0xf0, 0x83, 0x15, 0x6f],          "DWORD",    "01101111000101011000001111110000"),
])
def test_deserialize_known_values_bitarr_word(serialized: List[int], dtype: str, expected_value: str):
    assert expected_value == deserialize(serialized, base_data_type=dtype)


# ---------------------------------------------------------------------------
# Signed integers 8, 16, 24, 32, 40, 48, 56, 64 tests
# ---------------------------------------------------------------------------

# --- round-trip tests ---

@pytest.mark.parametrize("bit_width", [8, 16, 24, 32, 40, 48, 56, 64])
def test_roundtrip_random_sint(bit_width: int):
    """Serializing then deserializing a random valid value returns the original."""
    max_val = 1 << (bit_width-1)
    value = random.randint(-max_val, (max_val-1))
    dtype = f"INTEGER{bit_width}"
    assert value == deserialize(serialize(value, name=dtype), name=dtype)


@pytest.mark.parametrize("bit_width", [8, 16, 24, 32, 40, 48, 56, 64])
def test_roundtrip_zero_sint(bit_width: int):
    """Zero survives a round-trip for every integer size."""
    value = 0
    dtype = f"INTEGER{bit_width}"
    assert value == deserialize(serialize(value, name=dtype), name=dtype)


@pytest.mark.parametrize("bit_width", [8, 16, 24, 32, 40, 48, 56, 64])
def test_roundtrip_max_sint(bit_width: int):
    """max-value survives a round-trip for every integer size."""
    max_val = 1 << (bit_width-1)
    value = max_val-1
    dtype = f"INTEGER{bit_width}"
    assert value == deserialize(serialize(value, name=dtype), name=dtype)


@pytest.mark.parametrize("bit_width", [8, 16, 24, 32, 40, 48, 56, 64])
def test_roundtrip_min_sint(bit_width: int):
    """min-value survives a round-trip for every integer size."""
    max_val = 1 << (bit_width-1)
    value = -max_val
    dtype = f"INTEGER{bit_width}"
    assert value == deserialize(serialize(value, name=dtype), name=dtype)


# --- Explicit serialization checks (known input -> known list[int]) ---

@pytest.mark.parametrize("value, dtype, expected", [
    (-(1 << 3),   "INTEGER8",   [0xf8]),
    ((1 << 12),   "INTEGER16",  [0x00, 0x10]),
    (-(1 << 22),  "INTEGER24",  [0x00, 0x00, 0xc0]),
    ((1 << 2),    "INTEGER32",  [0x04, 0x00, 0x00, 0x00]),
    (-(1 << 20),  "INTEGER40",  [0x00, 0x00, 0xf0, 0xff, 0xff]),
    ((1 << 40),   "INTEGER48",  [0x00, 0x00, 0x00, 0x00, 0x00, 0x01]),
    (-(1 << 40),  "INTEGER56",  [0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff]),
    ((1 << 62),   "INTEGER64",  [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40]),
])
def test_serialize_known_values_sint(value: int, dtype: str, expected: List[int]):
    assert expected == serialize(value, name=dtype)


# --- Explicit deserialization checks (known list[int] -> known output) ---

@pytest.mark.parametrize("serialized, dtype, expected_value", [
    ([0xf8],                                         "INTEGER8",   -(1 << 3)),
    ([0x00, 0x10],                                   "INTEGER16",  (1 << 12)),
    ([0x00, 0x00, 0xc0],                             "INTEGER24",  -(1 << 22)),
    ([0x04, 0x00, 0x00, 0x00],                       "INTEGER32",  (1 << 2)),
    ([0x00, 0x00, 0xf0, 0xff, 0xff],                 "INTEGER40",  -(1 << 20)),
    ([0x00, 0x00, 0x00, 0x00, 0x00, 0x01],           "INTEGER48",  (1 << 40)),
    ([0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff],     "INTEGER56",  -(1 << 40)),
    ([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40], "INTEGER64", (1 << 62)),
])
def test_deserialize_known_values_sint(serialized: List[int], dtype: str, expected_value: int):
    assert expected_value == deserialize(serialized, name=dtype)


# ---------------------------------------------------------------------------
# Unsigned integers 8, 16, 24, 32, 40, 48, 56, 64 tests
# ---------------------------------------------------------------------------

# --- round-trip tests ---

@pytest.mark.parametrize("bit_width", [8, 16, 24, 32, 40, 48, 56, 64])
def test_roundtrip_random_uint(bit_width: int):
    """Serializing then deserializing a random valid value returns the original."""
    max_val = (1 << bit_width)-1
    value = random.randint(0, max_val)
    dtype = f"UNSIGNED{bit_width}"
    assert value == deserialize(serialize(value, name=dtype), name=dtype)


@pytest.mark.parametrize("bit_width", [8, 16, 24, 32, 40, 48, 56, 64])
def test_roundtrip_min_uint(bit_width: int):
    """Zero survives a round-trip for every unsigned integer size."""
    value = 0
    dtype = f"UNSIGNED{bit_width}"
    assert value == deserialize(serialize(value, name=dtype), name=dtype)


@pytest.mark.parametrize("bit_width", [8, 16, 24, 32, 40, 48, 56, 64])
def test_roundtrip_max_uint(bit_width: int):
    """max-value survives a round-trip for every unsigned integer size."""
    max_val = (1 << bit_width)-1
    value = max_val
    dtype = f"UNSIGNED{bit_width}"
    assert value == deserialize(serialize(value, name=dtype), name=dtype)


# ---------------------------------------------------------------------------
# Bool tests
# ---------------------------------------------------------------------------

# --- round-trip test ---

def test_roundtrip_random_bool():
    """Serializing then deserializing a random valid value returns the original."""
    value = random.choice([True, False, 1, 0])
    assert value == deserialize(serialize(value, base_data_type="BOOL"), base_data_type="BOOL")


# --- Explicit serialization checks (known input -> known list[int]) ---

@pytest.mark.parametrize("value, dtype, expected", [
    (False, "BOOL", [0x00]),
    (True,  "BOOL", [0x01]),
    (0,     "BOOL", [0x00]),
    (1,     "BOOL", [0x01]),
])
def test_serialize_known_values_bool(value, dtype: str, expected: List[int]):
    assert expected == serialize(value, base_data_type=dtype)


# --- Explicit deserialization checks (known list[int] -> known output) ---

@pytest.mark.parametrize("serialized, dtype, expected_value", [
    ([0x00], "BOOL", False),
    ([0x01], "BOOL", True),
    ([0x00], "BOOL", 0),
    ([0x01], "BOOL", 1),
])
def test_deserialize_known_values_bool(serialized: List[int], dtype: str, expected_value):
    assert expected_value == deserialize(serialized, base_data_type=dtype)


# ---------------------------------------------------------------------------
# Floating-point REAL32, REAL64 tests
# ---------------------------------------------------------------------------

# --- round-trip tests ---

@pytest.mark.parametrize(("dtype", "value"), [
    ("REAL32", random.uniform(-3.4e38, 3.4e38)),
    ("REAL64", random.uniform(-1.7e308, 1.7e308)),
])
def test_roundtrip_random_real(dtype: str, value: float):
    """Serializing then deserializing a random valid float returns the original."""
    result = deserialize(serialize(value, name=dtype), name=dtype)
    if dtype == "REAL32":
        assert result == pytest.approx(value, rel=1e-6) # Python uses 64 bit so we lose some precision on the roundtrip
    else:
        assert result == value


@pytest.mark.parametrize("dtype", ["REAL32", "REAL64"])
def test_roundtrip_zero_real(dtype: str):
    """Zero survives a round-trip for every floating-point size."""
    value = 0.0
    assert value == deserialize(serialize(value, name=dtype), name=dtype)


@pytest.mark.parametrize(("dtype", "value"), [
    ("REAL32", float("inf")),
    ("REAL32", float("-inf")),
    ("REAL64", float("inf")),
    ("REAL64", float("-inf")),
])
def test_roundtrip_infinity_real(dtype: str, value: float):
    """Infinity values survive a round-trip."""
    assert value == deserialize(serialize(value, name=dtype), name=dtype)


@pytest.mark.parametrize("dtype", ["REAL32", "REAL64"])
def test_roundtrip_nan_real(dtype: str):
    """NaN survives a round-trip."""
    value = float("nan")
    result = deserialize(serialize(value, name=dtype), name=dtype)
    assert isnan(result)