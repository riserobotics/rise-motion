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