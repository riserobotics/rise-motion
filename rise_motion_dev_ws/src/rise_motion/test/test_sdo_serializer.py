"""Tests for sdo_serializer.py module."""
import pytest
import random
from math import isnan
import uuid
from rise_motion.sdo_serializer import serialize, deserialize

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
def test_serialize_known_values_bitstrings(value: str, dtype: str, expected: list[int]):
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
def test_deserialize_known_values_bitstrings(serialized: list[int], dtype: str, expected_value: str):
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
def test_serialize_known_values_bitarr_word(value: str, dtype: str, expected: list[int]):
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
def test_deserialize_known_values_bitarr_word(serialized: list[int], dtype: str, expected_value: str):
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
def test_serialize_known_values_sint(value: int, dtype: str, expected: list[int]):
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
def test_deserialize_known_values_sint(serialized: list[int], dtype: str, expected_value: int):
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
    (True,  "BOOL", [0xff]),
    (0,     "BOOL", [0x00]),
    (1,     "BOOL", [0xff]),
])
def test_serialize_known_values_bool(value, dtype: str, expected: list[int]):
    assert expected == serialize(value, base_data_type=dtype)


# --- Explicit deserialization checks (known list[int] -> known output) ---

@pytest.mark.parametrize("serialized, dtype, expected_value", [
    ([0x00], "BOOL", False),
    ([0x01], "BOOL", True),
    ([0x00], "BOOL", 0),
    ([0x01], "BOOL", 1),
])
def test_deserialize_known_values_bool(serialized: list[int], dtype: str, expected_value):
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


# ---------------------------------------------------------------------------
# TIME_OF_DAY, TIME_DIFFERENCE tests
# ---------------------------------------------------------------------------

# --- round-trip tests ---

@pytest.mark.parametrize("dtype", ["TIME_OF_DAY", "TIME_DIFFERENCE"])
def test_roundtrip_random_time48(dtype: str):
    """Serializing then deserializing a random valid value returns the original."""
    ms = random.randint(0, (1 << 28) - 1)
    days = random.randint(0, (1 << 16) - 1)
    value = (ms, days)
    assert value == deserialize(serialize(value, base_data_type=dtype), base_data_type=dtype)


@pytest.mark.parametrize("dtype", ["TIME_OF_DAY", "TIME_DIFFERENCE"])
def test_roundtrip_min_time48(dtype: str):
    """Zero ms and days survive a round-trip for both data types."""
    assert (0, 0) == deserialize(serialize((0, 0), base_data_type=dtype), base_data_type=dtype)


@pytest.mark.parametrize("dtype", ["TIME_OF_DAY", "TIME_DIFFERENCE"])
def test_roundtrip_max_time48(dtype: str):
    """Max-value ms and days survive a round-trip for both data types."""
    ms = (1 << 28) - 1
    days = (1 << 16) - 1
    value = (ms, days)
    assert value == deserialize(serialize(value, base_data_type=dtype), base_data_type=dtype)


# --- Explicit serialization checks (known input -> known list[int]) ---

@pytest.mark.parametrize("value, dtype, expected", [
    ((5000, 5000), "TIME_OF_DAY", [0x00, 0x00, 0x13, 0x88, 0x13, 0x88]),
    ((1000, 1000), "TIME_DIFFERENCE", [0x00, 0x00, 0x03, 0xE8, 0x03, 0xE8]),
])
def test_serialize_known_values_time48(value: int, dtype: str, expected: list[int]):
    assert expected == serialize(value, name=dtype)


# --- Explicit deserialization checks (known list[int] -> known output) ---

@pytest.mark.parametrize("serialized, dtype, expected_value", [
    ([0x00, 0x00, 0x13, 0x88, 0x13, 0x88], "TIME_OF_DAY", (5000, 5000)),
    ([0x00, 0x00, 0x03, 0xE8, 0x03, 0xE8], "TIME_DIFFERENCE", (1000, 1000)),
])
def test_deserialize_known_values_time48(serialized: list[int], dtype: str, expected_value: int):
    assert expected_value == deserialize(serialized, name=dtype)


# ---------------------------------------------------------------------------
# GUID tests
# ---------------------------------------------------------------------------

# --- round-trip test ---

def test_roundtrip_random_guid():
    """Serializing then deserializing a random valid value returns the original."""
    max_val = 1 << (128-1)
    value = random.randint(-max_val, (max_val-1))
    assert value == deserialize(serialize(value, base_data_type="GUID"), base_data_type="GUID")


# ---------------------------------------------------------------------------
# VISIBLE_STRING (ASCII, 1 byte per char) tests
# ---------------------------------------------------------------------------

# --- round-trip tests ---

@pytest.mark.parametrize("value", [
    "hello",
    "Hello, World!",
    "EtherCAT",
    "test 123",
    "",                  # empty string
    "A",                 # single char
    " ",                 # space
    "!@#$%^&*()",        # special ASCII chars
    "a" * 100,           # long string
])
def test_roundtrip_visible_string(value: str):
    """Serializing then deserializing a string returns the original."""
    assert value == deserialize(serialize(value, name="VISIBLE_STRING"), name="VISIBLE_STRING")


# --- Explicit serialization checks (known input -> known list[int]) ---

@pytest.mark.parametrize("value, expected", [
    ("A",     [0x41]),
    ("AB",    [0x41, 0x42]),
    ("hi",    [0x68, 0x69]),
    ("",      []),
    ("\x00",  [0x00]),          # null byte
    (" ",     [0x20]),          # space
    ("ABC",   [0x41, 0x42, 0x43]),
])
def test_serialize_known_values_visible_string(value: str, expected: list[int]):
    assert expected == serialize(value, name="VISIBLE_STRING")


# --- Explicit deserialization checks (known list[int] -> known output) ---

@pytest.mark.parametrize("serialized, expected_value", [
    ([0x41],             "A"),
    ([0x41, 0x42],       "AB"),
    ([0x68, 0x69],       "hi"),
    ([],                 ""),
    ([0x00],             "\x00"),
    ([0x20],             " "),
    ([0x41, 0x42, 0x43], "ABC"),
])
def test_deserialize_known_values_visible_string(serialized: list[int], expected_value: str):
    assert expected_value == deserialize(serialized, name="VISIBLE_STRING")


# --- Invalid input ---

@pytest.mark.parametrize("value", [
    "café",      # non-ASCII (é is > 0x7F, invalid for VISIBLE_STRING)
    "日本語",    # CJK characters
    "😀",        # emoji
])
def test_serialize_visible_string_rejects_non_ascii(value: str):
    """Characters outside the visible ASCII range should raise."""
    with pytest.raises((ValueError, UnicodeEncodeError)):
        serialize(value, name="VISIBLE_STRING")


# ---------------------------------------------------------------------------
# UNICODE_STRING (UTF-16-LE, 2 bytes per char) tests
# ---------------------------------------------------------------------------

# --- round-trip tests ---

@pytest.mark.parametrize("value", [
    "hello",
    "Hello, World!",
    "",                  # empty string
    "A",                 # single char
    "café",              # accented characters
    "日本語",            # CJK characters
    "EtherCAT 🚀",       # emoji (surrogate pair in UTF-16)
    "a" * 100,           # long string
])
def test_roundtrip_unicode_string(value: str):
    """Serializing then deserializing a unicode string returns the original."""
    assert value == deserialize(serialize(value, name="UNICODE_STRING"), name="UNICODE_STRING")


# --- Explicit serialization checks (known input -> known list[int]) ---

@pytest.mark.parametrize("value, expected", [
    ("A",   [0x41, 0x00]),                          # U+0041, LE
    ("AB",  [0x41, 0x00, 0x42, 0x00]),              # two ASCII chars
    ("",    []),                                     # empty
    (" ",   [0x20, 0x00]),                           # space
    ("é",   [0xe9, 0x00]),                           # U+00E9
    ("中",  [0x2d, 0x4e]),                           # U+4E2D
])
def test_serialize_known_values_unicode_string(value: str, expected: list[int]):
    assert expected == serialize(value, name="UNICODE_STRING")


# --- Explicit deserialization checks (known list[int] -> known output) ---

@pytest.mark.parametrize("serialized, expected_value", [
    ([0x41, 0x00],             "A"),
    ([0x41, 0x00, 0x42, 0x00], "AB"),
    ([],                       ""),
    ([0x20, 0x00],             " "),
    ([0xe9, 0x00],             "é"),
    ([0x2d, 0x4e],             "中"),
])
def test_deserialize_known_values_unicode_string(serialized: list[int], expected_value: str):
    assert expected_value == deserialize(serialized, name="UNICODE_STRING")


# --- Byte count is always even ---

@pytest.mark.parametrize("value", ["A", "AB", "ABC", "日本語"])
def test_unicode_string_serialized_length_is_even(value: str):
    """UTF-16-LE encoding always produces an even number of bytes."""
    assert len(serialize(value, name="UNICODE_STRING")) % 2 == 0


# --- Odd-length input is rejected ---

def test_deserialize_unicode_string_rejects_odd_length():
    """An odd number of bytes cannot be valid UTF-16-LE."""
    with pytest.raises((ValueError, UnicodeDecodeError)):
        deserialize([0x41], name="UNICODE_STRING")


# ---------------------------------------------------------------------------
# OCTET_STRING and ARRAY_OF_BITARRn tests (arrays of bit strings)
# ---------------------------------------------------------------------------

# --- round-trip tests ---

@pytest.mark.parametrize("dtype, bit_width", [
    ("OCTET_STRING",      8),
    ("ARRAY_OF_BITARR8",  8),
    ("ARRAY_OF_BITARR16", 16),
    ("ARRAY_OF_BITARR32", 32),
])
def test_roundtrip_random_bitarr_array(dtype: str, bit_width: int):
    """Serializing then deserializing a random array of bit strings returns the original."""
    max_val = (1 << bit_width) - 1
    value = [format(random.randint(0, max_val), f"0{bit_width}b") for _ in range(random.randint(1, 8))]
    assert value == deserialize(serialize(value, name=dtype), name=dtype)


@pytest.mark.parametrize("dtype, bit_width", [
    ("OCTET_STRING",      8),
    ("ARRAY_OF_BITARR8",  8),
    ("ARRAY_OF_BITARR16", 16),
    ("ARRAY_OF_BITARR32", 32),
])
def test_roundtrip_single_element_bitarr_array(dtype: str, bit_width: int):
    """Single-element array survives a round-trip."""
    value = ["0" * bit_width]
    assert value == deserialize(serialize(value, name=dtype), name=dtype)


@pytest.mark.parametrize("dtype, bit_width", [
    ("OCTET_STRING",      8),
    ("ARRAY_OF_BITARR8",  8),
    ("ARRAY_OF_BITARR16", 16),
    ("ARRAY_OF_BITARR32", 32),
])
def test_roundtrip_all_ones_bitarr_array(dtype: str, bit_width: int):
    """All-ones array survives a round-trip."""
    value = ["1" * bit_width] * 4
    assert value == deserialize(serialize(value, name=dtype), name=dtype)


# --- Explicit serialization checks ---

@pytest.mark.parametrize("value, dtype, expected", [
    # OCTET_STRING / ARRAY_OF_BITARR8: 1 byte per element
    (["10101100"],                         "OCTET_STRING",      [0xac]),
    (["10101100", "01010101"],             "OCTET_STRING",      [0xac, 0x55]),
    (["00000000", "11111111"],             "OCTET_STRING",      [0x00, 0xff]),
    (["10101100"],                         "ARRAY_OF_BITARR8",  [0xac]),
    (["10101100", "01010101"],             "ARRAY_OF_BITARR8",  [0xac, 0x55]),
    # ARRAY_OF_BITARR16: 2 bytes per element
    (["1100101001110001"],                 "ARRAY_OF_BITARR16", [0x71, 0xca]),
    (["1100101001110001", "0011110000101110"], "ARRAY_OF_BITARR16", [0x71, 0xca, 0x2e, 0x3c]),
    # ARRAY_OF_BITARR32: 4 bytes per element
    (["10010000111100001010101001101101"], "ARRAY_OF_BITARR32", [0x6d, 0xaa, 0xf0, 0x90]),
    (["10010000111100001010101001101101",
       "01101111000101011000001111110000"], "ARRAY_OF_BITARR32", [0x6d, 0xaa, 0xf0, 0x90,
                                                                  0xf0, 0x83, 0x15, 0x6f]),
])
def test_serialize_known_values_bitarr_array(value: list, dtype: str, expected: list[int]):
    assert expected == serialize(value, name=dtype)


# --- Explicit deserialization checks ---

@pytest.mark.parametrize("serialized, dtype, expected_value", [
    ([0xac],             "OCTET_STRING",      ["10101100"]),
    ([0xac, 0x55],       "OCTET_STRING",      ["10101100", "01010101"]),
    ([0x00, 0xff],       "OCTET_STRING",      ["00000000", "11111111"]),
    ([0xac],             "ARRAY_OF_BITARR8",  ["10101100"]),
    ([0xac, 0x55],       "ARRAY_OF_BITARR8",  ["10101100", "01010101"]),
    ([0x71, 0xca],       "ARRAY_OF_BITARR16", ["1100101001110001"]),
    ([0x71, 0xca, 0x2e, 0x3c], "ARRAY_OF_BITARR16", ["1100101001110001", "0011110000101110"]),
    ([0x6d, 0xaa, 0xf0, 0x90], "ARRAY_OF_BITARR32", ["10010000111100001010101001101101"]),
    ([0x6d, 0xaa, 0xf0, 0x90,
      0xf0, 0x83, 0x15, 0x6f], "ARRAY_OF_BITARR32", ["10010000111100001010101001101101",
                                                       "01101111000101011000001111110000"]),
])
def test_deserialize_known_values_bitarr_array(serialized: list[int], dtype: str, expected_value: list):
    assert expected_value == deserialize(serialized, name=dtype)


# ---------------------------------------------------------------------------
# ARRAY_OF_UINT, ARRAY_OF_UDINT, ARRAY_OF_USINT tests (unsigned int arrays)
# ---------------------------------------------------------------------------

# --- round-trip tests ---

@pytest.mark.parametrize("dtype, bit_width", [
    ("ARRAY_OF_USINT", 8),
    ("ARRAY_OF_UINT",  16),
    ("ARRAY_OF_UDINT", 32),
])
def test_roundtrip_random_uint_array(dtype: str, bit_width: int):
    """Serializing then deserializing a random array of uints returns the original."""
    max_val = (1 << bit_width) - 1
    value = [random.randint(0, max_val) for _ in range(random.randint(1, 8))]
    assert value == deserialize(serialize(value, name=dtype), name=dtype)


@pytest.mark.parametrize("dtype", ["ARRAY_OF_USINT", "ARRAY_OF_UINT", "ARRAY_OF_UDINT"])
def test_roundtrip_zeros_uint_array(dtype: str):
    """Array of zeros survives a round-trip."""
    value = [0, 0, 0, 0]
    assert value == deserialize(serialize(value, name=dtype), name=dtype)


@pytest.mark.parametrize("dtype, bit_width", [
    ("ARRAY_OF_USINT", 8),
    ("ARRAY_OF_UINT",  16),
    ("ARRAY_OF_UDINT", 32),
])
def test_roundtrip_max_uint_array(dtype: str, bit_width: int):
    """Array of max values survives a round-trip."""
    max_val = (1 << bit_width) - 1
    value = [max_val] * 4
    assert value == deserialize(serialize(value, name=dtype), name=dtype)


# --- Explicit serialization checks ---

@pytest.mark.parametrize("value, dtype, expected", [
    # ARRAY_OF_USINT: 1 byte per element
    ([0],          "ARRAY_OF_USINT", [0x00]),
    ([255],        "ARRAY_OF_USINT", [0xff]),
    ([1, 2, 3],    "ARRAY_OF_USINT", [0x01, 0x02, 0x03]),
    ([0, 128, 255],"ARRAY_OF_USINT", [0x00, 0x80, 0xff]),
    # ARRAY_OF_UINT: 2 bytes per element (little-endian)
    ([0],          "ARRAY_OF_UINT",  [0x00, 0x00]),
    ([1],          "ARRAY_OF_UINT",  [0x01, 0x00]),
    ([256],        "ARRAY_OF_UINT",  [0x00, 0x01]),
    ([1, 2],       "ARRAY_OF_UINT",  [0x01, 0x00, 0x02, 0x00]),
    ([0x1234],     "ARRAY_OF_UINT",  [0x34, 0x12]),
    # ARRAY_OF_UDINT: 4 bytes per element (little-endian)
    ([0],          "ARRAY_OF_UDINT", [0x00, 0x00, 0x00, 0x00]),
    ([1],          "ARRAY_OF_UDINT", [0x01, 0x00, 0x00, 0x00]),
    ([0x12345678], "ARRAY_OF_UDINT", [0x78, 0x56, 0x34, 0x12]),
    ([1, 2],       "ARRAY_OF_UDINT", [0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00]),
])
def test_serialize_known_values_uint_array(value: list, dtype: str, expected: list[int]):
    assert expected == serialize(value, name=dtype)


# --- Explicit deserialization checks ---

@pytest.mark.parametrize("serialized, dtype, expected_value", [
    ([0x00],                               "ARRAY_OF_USINT", [0]),
    ([0xff],                               "ARRAY_OF_USINT", [255]),
    ([0x01, 0x02, 0x03],                   "ARRAY_OF_USINT", [1, 2, 3]),
    ([0x00, 0x80, 0xff],                   "ARRAY_OF_USINT", [0, 128, 255]),
    ([0x00, 0x00],                         "ARRAY_OF_UINT",  [0]),
    ([0x01, 0x00],                         "ARRAY_OF_UINT",  [1]),
    ([0x34, 0x12],                         "ARRAY_OF_UINT",  [0x1234]),
    ([0x01, 0x00, 0x02, 0x00],             "ARRAY_OF_UINT",  [1, 2]),
    ([0x00, 0x00, 0x00, 0x00],             "ARRAY_OF_UDINT", [0]),
    ([0x78, 0x56, 0x34, 0x12],             "ARRAY_OF_UDINT", [0x12345678]),
    ([0x01, 0x00, 0x00, 0x00,
      0x02, 0x00, 0x00, 0x00],             "ARRAY_OF_UDINT", [1, 2]),
])
def test_deserialize_known_values_uint_array(serialized: list[int], dtype: str, expected_value: list):
    assert expected_value == deserialize(serialized, name=dtype)


# ---------------------------------------------------------------------------
# ARRAY_OF_INT, ARRAY_OF_SINT, ARRAY_OF_DINT tests (signed int arrays)
# ---------------------------------------------------------------------------

# --- round-trip tests ---

@pytest.mark.parametrize("dtype, bit_width", [
    ("ARRAY_OF_SINT", 8),
    ("ARRAY_OF_INT",  16),
    ("ARRAY_OF_DINT", 32),
])
def test_roundtrip_random_sint_array(dtype: str, bit_width: int):
    """Serializing then deserializing a random array of signed ints returns the original."""
    half = 1 << (bit_width - 1)
    value = [random.randint(-half, half - 1) for _ in range(random.randint(1, 8))]
    assert value == deserialize(serialize(value, name=dtype), name=dtype)


@pytest.mark.parametrize("dtype", ["ARRAY_OF_SINT", "ARRAY_OF_INT", "ARRAY_OF_DINT"])
def test_roundtrip_zeros_sint_array(dtype: str):
    """Array of zeros survives a round-trip."""
    value = [0, 0, 0, 0]
    assert value == deserialize(serialize(value, name=dtype), name=dtype)


@pytest.mark.parametrize("dtype, bit_width", [
    ("ARRAY_OF_SINT", 8),
    ("ARRAY_OF_INT",  16),
    ("ARRAY_OF_DINT", 32),
])
def test_roundtrip_min_max_sint_array(dtype: str, bit_width: int):
    """Array of min and max values survives a round-trip."""
    half = 1 << (bit_width - 1)
    value = [-half, half - 1]
    assert value == deserialize(serialize(value, name=dtype), name=dtype)


# --- Explicit serialization checks ---

@pytest.mark.parametrize("value, dtype, expected", [
    # ARRAY_OF_SINT: 1 byte per element (signed)
    ([0],        "ARRAY_OF_SINT", [0x00]),
    ([-1],       "ARRAY_OF_SINT", [0xff]),
    ([127],      "ARRAY_OF_SINT", [0x7f]),
    ([-128],     "ARRAY_OF_SINT", [0x80]),
    ([-1, 1],    "ARRAY_OF_SINT", [0xff, 0x01]),
    # ARRAY_OF_INT: 2 bytes per element (signed, little-endian)
    ([0],        "ARRAY_OF_INT",  [0x00, 0x00]),
    ([-1],       "ARRAY_OF_INT",  [0xff, 0xff]),
    ([1],        "ARRAY_OF_INT",  [0x01, 0x00]),
    ([-1, 1],    "ARRAY_OF_INT",  [0xff, 0xff, 0x01, 0x00]),
    ([(1 << 12)],"ARRAY_OF_INT",  [0x00, 0x10]),
    # ARRAY_OF_DINT: 4 bytes per element (signed, little-endian)
    ([0],        "ARRAY_OF_DINT", [0x00, 0x00, 0x00, 0x00]),
    ([-1],       "ARRAY_OF_DINT", [0xff, 0xff, 0xff, 0xff]),
    ([1],        "ARRAY_OF_DINT", [0x01, 0x00, 0x00, 0x00]),
    ([-1, 1],    "ARRAY_OF_DINT", [0xff, 0xff, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00]),
])
def test_serialize_known_values_sint_array(value: list, dtype: str, expected: list[int]):
    assert expected == serialize(value, name=dtype)


# --- Explicit deserialization checks ---

@pytest.mark.parametrize("serialized, dtype, expected_value", [
    ([0x00],                               "ARRAY_OF_SINT", [0]),
    ([0xff],                               "ARRAY_OF_SINT", [-1]),
    ([0x7f],                               "ARRAY_OF_SINT", [127]),
    ([0x80],                               "ARRAY_OF_SINT", [-128]),
    ([0xff, 0x01],                         "ARRAY_OF_SINT", [-1, 1]),
    ([0x00, 0x00],                         "ARRAY_OF_INT",  [0]),
    ([0xff, 0xff],                         "ARRAY_OF_INT",  [-1]),
    ([0xff, 0xff, 0x01, 0x00],             "ARRAY_OF_INT",  [-1, 1]),
    ([0x00, 0x10],                         "ARRAY_OF_INT",  [1 << 12]),
    ([0x00, 0x00, 0x00, 0x00],             "ARRAY_OF_DINT", [0]),
    ([0xff, 0xff, 0xff, 0xff],             "ARRAY_OF_DINT", [-1]),
    ([0xff, 0xff, 0xff, 0xff,
      0x01, 0x00, 0x00, 0x00],             "ARRAY_OF_DINT", [-1, 1]),
])
def test_deserialize_known_values_sint_array(serialized: list[int], dtype: str, expected_value: list):
    assert expected_value == deserialize(serialized, name=dtype)


# ---------------------------------------------------------------------------
# ARRAY_OF_REAL, ARRAY_OF_LREAL tests (float arrays)
# ---------------------------------------------------------------------------

# --- round-trip tests ---

@pytest.mark.parametrize("dtype, gen", [
    ("ARRAY_OF_REAL",  lambda: random.uniform(-3.4e38,  3.4e38)),
    ("ARRAY_OF_LREAL", lambda: random.uniform(-1.7e308, 1.7e308)),
])
def test_roundtrip_random_float_array(dtype: str, gen):
    """Serializing then deserializing a random float array returns the original."""
    value = [gen() for _ in range(random.randint(1, 8))]
    result = deserialize(serialize(value, name=dtype), name=dtype)
    if dtype == "ARRAY_OF_REAL":
        assert result == pytest.approx(value, rel=1e-6)
    else:
        assert result == value


@pytest.mark.parametrize("dtype", ["ARRAY_OF_REAL", "ARRAY_OF_LREAL"])
def test_roundtrip_zeros_float_array(dtype: str):
    """Array of zeros survives a round-trip."""
    value = [0.0, 0.0, 0.0]
    assert deserialize(serialize(value, name=dtype), name=dtype) == value


@pytest.mark.parametrize("dtype", ["ARRAY_OF_REAL", "ARRAY_OF_LREAL"])
def test_roundtrip_inf_float_array(dtype: str):
    """Infinity values survive a round-trip."""
    value = [float("inf"), float("-inf")]
    assert deserialize(serialize(value, name=dtype), name=dtype) == value


@pytest.mark.parametrize("dtype", ["ARRAY_OF_REAL", "ARRAY_OF_LREAL"])
def test_roundtrip_nan_float_array(dtype: str):
    """NaN values survive a round-trip."""
    value = [float("nan"), float("nan")]
    result = deserialize(serialize(value, name=dtype), name=dtype)
    assert all(isnan(r) for r in result)


# --- Explicit serialization checks ---

@pytest.mark.parametrize("value, dtype, expected", [
    # ARRAY_OF_REAL: 4 bytes per element
    ([0.0],        "ARRAY_OF_REAL",  [0x00, 0x00, 0x00, 0x00]),
    ([1.0],        "ARRAY_OF_REAL",  [0x00, 0x00, 0x80, 0x3f]),
    ([-1.0],       "ARRAY_OF_REAL",  [0x00, 0x00, 0x80, 0xbf]),
    ([0.0, 1.0],   "ARRAY_OF_REAL",  [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3f]),
    # ARRAY_OF_LREAL: 8 bytes per element
    ([0.0],        "ARRAY_OF_LREAL", [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
    ([1.0],        "ARRAY_OF_LREAL", [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x3f]),
    ([-1.0],       "ARRAY_OF_LREAL", [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xbf]),
])
def test_serialize_known_values_float_array(value: list, dtype: str, expected: list[int]):
    assert expected == serialize(value, name=dtype)


# --- Explicit deserialization checks ---

@pytest.mark.parametrize("serialized, dtype, expected_value", [
    ([0x00, 0x00, 0x00, 0x00],             "ARRAY_OF_REAL",  [0.0]),
    ([0x00, 0x00, 0x80, 0x3f],             "ARRAY_OF_REAL",  [1.0]),
    ([0x00, 0x00, 0x80, 0xbf],             "ARRAY_OF_REAL",  [-1.0]),
    ([0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x80, 0x3f],             "ARRAY_OF_REAL",  [0.0, 1.0]),
    ([0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00],             "ARRAY_OF_LREAL", [0.0]),
    ([0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0xf0, 0x3f],             "ARRAY_OF_LREAL", [1.0]),
    ([0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0xf0, 0xbf],             "ARRAY_OF_LREAL", [-1.0]),
])
def test_deserialize_known_values_float_array(serialized: list[int], dtype: str, expected_value: list):
    assert expected_value == pytest.approx(deserialize(serialized, name=dtype))


# ---------------------------------------------------------------------------
# General array property tests (all array types)
# ---------------------------------------------------------------------------

ALL_ARRAY_TYPES = [
    "OCTET_STRING",
    "ARRAY_OF_UINT", "ARRAY_OF_INT", "ARRAY_OF_SINT", "ARRAY_OF_DINT", "ARRAY_OF_UDINT",
    "ARRAY_OF_BITARR8", "ARRAY_OF_BITARR16", "ARRAY_OF_BITARR32",
    "ARRAY_OF_USINT", "ARRAY_OF_REAL", "ARRAY_OF_LREAL",
]

ELEMENT_BYTE_WIDTH = {
    "OCTET_STRING":      1,
    "ARRAY_OF_USINT":    1, "ARRAY_OF_SINT":    1, "ARRAY_OF_BITARR8":  1,
    "ARRAY_OF_UINT":     2, "ARRAY_OF_INT":     2, "ARRAY_OF_BITARR16": 2,
    "ARRAY_OF_UDINT":    4, "ARRAY_OF_DINT":    4, "ARRAY_OF_BITARR32": 4, "ARRAY_OF_REAL": 4,
    "ARRAY_OF_LREAL":    8,
}

ELEMENT_GENERATORS = {
    "OCTET_STRING":      lambda: format(random.randint(0, 0xff), "08b"),
    "ARRAY_OF_USINT":    lambda: random.randint(0, 0xff),
    "ARRAY_OF_UINT":     lambda: random.randint(0, 0xffff),
    "ARRAY_OF_UDINT":    lambda: random.randint(0, 0xffffffff),
    "ARRAY_OF_SINT":     lambda: random.randint(-128, 127),
    "ARRAY_OF_INT":      lambda: random.randint(-32768, 32767),
    "ARRAY_OF_DINT":     lambda: random.randint(-(1 << 31), (1 << 31) - 1),
    "ARRAY_OF_BITARR8":  lambda: format(random.randint(0, 0xff), "08b"),
    "ARRAY_OF_BITARR16": lambda: format(random.randint(0, 0xffff), "016b"),
    "ARRAY_OF_BITARR32": lambda: format(random.randint(0, 0xffffffff), "032b"),
    "ARRAY_OF_REAL":     lambda: random.uniform(-1e10, 1e10),
    "ARRAY_OF_LREAL":    lambda: random.uniform(-1e100, 1e100),
}


@pytest.mark.parametrize("dtype", ALL_ARRAY_TYPES)
def test_serialized_length_matches_element_count(dtype: str):
    """Serialized byte count equals number of elements × bytes per element."""
    n = random.randint(1, 8)
    gen = ELEMENT_GENERATORS[dtype]
    value = [gen() for _ in range(n)]
    result = serialize(value, name=dtype)
    assert len(result) == n * ELEMENT_BYTE_WIDTH[dtype]


@pytest.mark.parametrize("dtype", ALL_ARRAY_TYPES)
def test_empty_array_serializes_to_empty(dtype: str):
    """An empty array serializes to an empty list."""
    assert [] == serialize([], name=dtype)


@pytest.mark.parametrize("dtype", ALL_ARRAY_TYPES)
def test_empty_array_deserializes_to_empty(dtype: str):
    """An empty list deserializes to an empty array."""
    assert [] == deserialize([], name=dtype)