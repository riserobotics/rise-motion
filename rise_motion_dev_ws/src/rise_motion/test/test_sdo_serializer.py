"""Tests for sdo_serializer.py module."""
import random
import pytest
from rise_motion.sdo_serializer import serialize, deserialize

# ---------------------------------------------------------------------------
# Bit strings BIT1 - BIT16 tests
# ---------------------------------------------------------------------------

# --- round-trip tests ---

@pytest.mark.parametrize("bit_width", range(1, 17))
def test_roundtrip_random(bit_width: int):
    """Serializing then deserializing a random valid value returns the original."""
    max_val = (1 << bit_width) - 1
    value = format(random.randint(0, max_val), f"0{bit_width}b")
    dtype = f"BIT{bit_width}"
    assert value == deserialize(serialize(value, base_data_type=dtype), base_data_type=dtype)


@pytest.mark.parametrize("bit_width", range(1, 17))
def test_roundtrip_min(bit_width: int):
    """Zero survives a round-trip for every bit width."""
    value = "0".zfill(bit_width)
    dtype = f"BIT{bit_width}"
    assert value == deserialize(serialize(value, base_data_type=dtype), base_data_type=dtype)


@pytest.mark.parametrize("bit_width", range(1, 17))
def test_roundtrip_max(bit_width: int):
    """All-ones value survives a round-trip for every bit width."""
    value = "1" * bit_width
    dtype = f"BIT{bit_width}"
    assert value == deserialize(serialize(value, base_data_type=dtype), base_data_type=dtype)


# --- Explicit serialization checks (known input -> known bytes) ---

@pytest.mark.parametrize("value, dtype, expected_bytes", [
    ("0",                "BIT1",  b"\x00"),
    ("1",                "BIT1",  b"\x01"),
    ("11",               "BIT2",  b"\x03"),
    ("10",               "BIT2",  b"\x02"),
    ("1111111111111111", "BIT16", b"\xff\xff"),
    ("0000000000000000", "BIT16", b"\x00\x00"),
    ("1000000000000000", "BIT16", b"\x00\x80"),  # MSB only
])
def test_serialize_known_values(value: str, dtype: str, expected_bytes: bytes):
    assert expected_bytes == serialize(value, base_data_type=dtype)


# --- Explicit deserialization checks (known bytes -> known output) ---

@pytest.mark.parametrize("raw_bytes, dtype, expected_value", [
    (b"\x00", "BIT1",  "0"),
    (b"\x01", "BIT1",  "1"),
    (b"\x03", "BIT2",  "11"),
    (b"\x02", "BIT2",  "10"),
    (b"\xff\xff", "BIT16", "1111111111111111"),
    (b"\x00\x00", "BIT16", "0000000000000000"),
])
def test_deserialize_known_values(raw_bytes: bytes, dtype: str, expected_value: str):
    assert expected_value == deserialize(raw_bytes, base_data_type=dtype)


# ---------------------------------------------------------------------------
# Bit arrays BITARR8, BITARR16, BITARR32 tests
# ---------------------------------------------------------------------------

# --- round-trip tests ---

@pytest.mark.parametrize("bit_width", [8, 16, 32])
def test_roundtrip_random(bit_width: int):
    """Serializing then deserializing a random valid value returns the original."""
    max_val = (1 << bit_width) - 1
    value = format(random.randint(0, max_val), f"0{bit_width}b")
    dtype = f"BITARR{bit_width}"
    assert value == deserialize(serialize(value, base_data_type=dtype), base_data_type=dtype)


@pytest.mark.parametrize("bit_width", [8, 16, 32])
def test_roundtrip_min(bit_width: int):
    """Zero survives a round-trip for every bit width."""
    value = "0".zfill(bit_width)
    dtype = f"BITARR{bit_width}"
    assert value == deserialize(serialize(value, base_data_type=dtype), base_data_type=dtype)


@pytest.mark.parametrize("bit_width", [8, 16, 32])
def test_roundtrip_max(bit_width: int):
    """All-ones value survives a round-trip for every bit width."""
    value = "1" * bit_width
    dtype = f"BITARR{bit_width}"
    assert value == deserialize(serialize(value, base_data_type=dtype), base_data_type=dtype)


# --- Explicit serialization checks (known input -> known bytes) ---

@pytest.mark.parametrize("value, dtype, expected_bytes", [
    ("10101100",                         "BITARR8",  b"\xac"),
    ("01010101",                         "BITARR8",  b"\x55"),
    ("1100101001110001",                 "BITARR16", b"\x71\xca"),
    ("0011110000101110",                 "BITARR16", b"\x2e\x3c"),
    ("10010000111100001010101001101101", "BITARR32", b"\x6d\xaa\xf0\x90"),
    ("01101111000101011000001111110000", "BITARR32", b"\xf0\x83\x15\x6f"),
])
def test_serialize_known_values(value: str, dtype: str, expected_bytes: bytes):
    assert expected_bytes == serialize(value, base_data_type=dtype)


# --- Explicit deserialization checks (known bytes -> known output) ---

@pytest.mark.parametrize("expected_value, dtype, raw_bytes", [
    ("10101100",                         "BITARR8",  b"\xac"),
    ("01010101",                         "BITARR8",  b"\x55"),
    ("1100101001110001",                 "BITARR16", b"\x71\xca"),
    ("0011110000101110",                 "BITARR16", b"\x2e\x3c"),
    ("10010000111100001010101001101101", "BITARR32", b"\x6d\xaa\xf0\x90"),
    ("01101111000101011000001111110000", "BITARR32", b"\xf0\x83\x15\x6f"),
])
def test_deserialize_known_values(raw_bytes: bytes, dtype: str, expected_value: str):
    assert expected_value == deserialize(raw_bytes, base_data_type=dtype)


# ---------------------------------------------------------------------------
# Signed integers 8, 16, 24, 32, 40, 48, 56, 64 tests
# ---------------------------------------------------------------------------
# These tests could be overkill for such a simple function,
# because I wrote them using the tests for the datatypes above as a template.
# Maybe they will help should the serialization for signed ints get more elaborate.

# --- round-trip tests ---

@pytest.mark.parametrize("bit_width", [8, 16, 24, 32, 40, 48, 56, 64])
def test_roundtrip_random(bit_width: int):
    """Serializing then deserializing a random valid value returns the original."""
    max_val = 1 << (bit_width-1)
    value = random.randint(-max_val, (max_val-1))
    dtype = f"INTEGER{bit_width}"
    assert value == deserialize(serialize(value, name=dtype), name=dtype)


@pytest.mark.parametrize("bit_width", [8, 16, 24, 32, 40, 48, 56, 64])
def test_roundtrip_min(bit_width: int):
    """Zero survives a round-trip for every integer size."""
    value = 0
    dtype = f"INTEGER{bit_width}"
    assert value == deserialize(serialize(value, name=dtype), name=dtype)


@pytest.mark.parametrize("bit_width", [8, 16, 24, 32, 40, 48, 56, 64])
def test_roundtrip_max(bit_width: int):
    """max-value survives a round-trip for every integer size."""
    max_val = 1 << (bit_width-1)
    value = max_val-1
    dtype = f"INTEGER{bit_width}"
    assert value == deserialize(serialize(value, name=dtype), name=dtype)


@pytest.mark.parametrize("bit_width", [8, 16, 24, 32, 40, 48, 56, 64])
def test_roundtrip_max(bit_width: int):
    """min-value survives a round-trip for every integer size."""
    max_val = 1 << (bit_width-1)
    value = -max_val
    dtype = f"INTEGER{bit_width}"
    assert value == deserialize(serialize(value, name=dtype), name=dtype)


# --- Explicit serialization checks (known input -> known bytes) ---

@pytest.mark.parametrize("value, dtype, expected_bytes", [
    (-(1 << 3),   "INTEGER8",   b"\xf8"),
    ((1 << 12),   "INTEGER16",  b"\x00\x10"),
    (-(1 << 22),  "INTEGER24",  b"\x00\x00\xc0"),
    ((1 << 2),    "INTEGER32",  b"\x04\x00\x00\x00"),
    (-(1 << 20),  "INTEGER40",  b"\x00\x00\xf0\xff\xff"),
    ((1 << 40),   "INTEGER48",  b"\x00\x00\x00\x00\x00\x01"),
    (-(1 << 40),  "INTEGER56",  b"\x00\x00\x00\x00\x00\xff\xff"),
    ((1 << 62),   "INTEGER64",  b"\x00\x00\x00\x00\x00\x00\x00\x40"),
])
def test_serialize_known_values(value: str, dtype: str, expected_bytes: bytes):
    assert expected_bytes == serialize(value, name=dtype)


# --- Explicit deserialization checks (known bytes -> known output) ---

@pytest.mark.parametrize("expected_value, dtype, raw_bytes", [
    (-(1 << 3),   "INTEGER8",   b"\xf8"),
    ((1 << 12),   "INTEGER16",  b"\x00\x10"),
    (-(1 << 22),  "INTEGER24",  b"\x00\x00\xc0"),
    ((1 << 2),    "INTEGER32",  b"\x04\x00\x00\x00"),
    (-(1 << 20),  "INTEGER40",  b"\x00\x00\xf0\xff\xff"),
    ((1 << 40),   "INTEGER48",  b"\x00\x00\x00\x00\x00\x01"),
    (-(1 << 40),  "INTEGER56",  b"\x00\x00\x00\x00\x00\xff\xff"),
    ((1 << 62),   "INTEGER64",  b"\x00\x00\x00\x00\x00\x00\x00\x40"),
])
def test_deserialize_known_values(raw_bytes: bytes, dtype: str, expected_value: str):
    assert expected_value == deserialize(raw_bytes, name=dtype)