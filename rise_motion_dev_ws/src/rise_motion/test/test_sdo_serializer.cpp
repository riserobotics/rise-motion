#include <gtest/gtest.h>

#include "rise_motion/sdo_serializer.hpp"

TEST(SDOSerializationTest, Bool)
{
  bool value = true;

  std::vector<std::uint8_t> blob = sdo::serialize<bool>(value);
  auto result = sdo::deserialize<bool>(blob);

  ASSERT_EQ(result, value);
}

TEST(SDOSerializationTest, Int8)
{
  std::int8_t value_zero = 0;
  std::int8_t value_max = 127;
  std::int8_t value_min = -128;

  std::vector<std::uint8_t> blob = sdo::serialize<std::int8_t>(value_zero);
  auto result_zero = sdo::deserialize<std::int8_t>(blob);
  ASSERT_EQ(result_zero, value_zero);

  blob = sdo::serialize<std::int8_t>(value_max);
  auto result_max = sdo::deserialize<std::int8_t>(blob);
  ASSERT_EQ(result_max, value_max);

  blob = sdo::serialize<std::int8_t>(value_min);
  auto result_min = sdo::deserialize<std::int8_t>(blob);
  ASSERT_EQ(result_min, value_min);
}