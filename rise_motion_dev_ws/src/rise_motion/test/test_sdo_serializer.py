import random
from rise_motion.sdo_serializer import *

def test_SDO_serialization_bit1():
    value = random.choice(('0', '1'))
    assert value == deserialize(serialize(value, base_data_type="BIT1"), base_data_type="BIT1")

def test_SDO_serialization_bit2():
    value = format(random.randint(0, 3), "b")
    assert value == deserialize(serialize(value, base_data_type="BIT1"), base_data_type="BIT1")