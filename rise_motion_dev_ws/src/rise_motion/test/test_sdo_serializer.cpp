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
  std::int8_t value_max = std::numeric_limits<std::int8_t>::max();
  std::int8_t value_min = std::numeric_limits<std::int8_t>::min();

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

TEST(SDOSerializationTest, Int16)
{
  std::int16_t value_zero = 0;
  std::int16_t value_max = std::numeric_limits<std::int16_t>::max();
  std::int16_t value_min = std::numeric_limits<std::int16_t>::min();

  std::vector<std::uint8_t> blob = sdo::serialize<std::int16_t>(value_zero);
  auto result_zero = sdo::deserialize<std::int16_t>(blob);
  ASSERT_EQ(result_zero, value_zero);

  blob = sdo::serialize<std::int16_t>(value_max);
  auto result_max = sdo::deserialize<std::int16_t>(blob);
  ASSERT_EQ(result_max, value_max);

  blob = sdo::serialize<std::int16_t>(value_min);
  auto result_min = sdo::deserialize<std::int16_t>(blob);
  ASSERT_EQ(result_min, value_min);
}

TEST(SDOSerializationTest, Int32)
{
  std::int32_t value_zero = 0;
  std::int32_t value_max = std::numeric_limits<std::int32_t>::max();
  std::int32_t value_min = std::numeric_limits<std::int32_t>::min();

  std::vector<std::uint8_t> blob = sdo::serialize<std::int32_t>(value_zero);
  auto result_zero = sdo::deserialize<std::int32_t>(blob);
  ASSERT_EQ(result_zero, value_zero);

  blob = sdo::serialize<std::int32_t>(value_max);
  auto result_max = sdo::deserialize<std::int32_t>(blob);
  ASSERT_EQ(result_max, value_max);

  blob = sdo::serialize<std::int32_t>(value_min);
  auto result_min = sdo::deserialize<std::int32_t>(blob);
  ASSERT_EQ(result_min, value_min);
}

TEST(SDOSerializationTest, uInt8)
{
  std::uint8_t value_zero = 0;
  std::uint8_t value_max = std::numeric_limits<std::uint8_t>::max();

  std::vector<std::uint8_t> blob = sdo::serialize<sdo::USINT>(value_zero);
  auto result_zero = sdo::deserialize<sdo::UNSIGNED8>(blob);
  ASSERT_EQ(result_zero, value_zero);

  blob = sdo::serialize<std::uint8_t>(value_max);
  auto result_max = sdo::deserialize<std::uint8_t>(blob);
  ASSERT_EQ(result_max, value_max);
}

TEST(SDOSerializationTest, uInt16)
{
  std::uint16_t value_zero = 0;
  std::uint16_t value_max = std::numeric_limits<std::uint16_t>::max();

  std::vector<std::uint8_t> blob = sdo::serialize<std::uint16_t>(value_zero);
  auto result_zero = sdo::deserialize<std::uint16_t>(blob);
  ASSERT_EQ(result_zero, value_zero);

  blob = sdo::serialize<std::uint16_t>(value_max);
  auto result_max = sdo::deserialize<std::uint16_t>(blob);
  ASSERT_EQ(result_max, value_max);
}

TEST(SDOSerializationTest, uInt32)
{
  std::uint32_t value_zero = 0;
  std::uint32_t value_max = std::numeric_limits<std::uint32_t>::max();

  std::vector<std::uint8_t> blob = sdo::serialize<std::uint32_t>(value_zero);
  auto result_zero = sdo::deserialize<std::uint32_t>(blob);
  ASSERT_EQ(result_zero, value_zero);

  blob = sdo::serialize<std::uint32_t>(value_max);
  auto result_max = sdo::deserialize<std::uint32_t>(blob);
  ASSERT_EQ(result_max, value_max);
}

TEST(SDOSerializationTest, Float)
{
  float value_zero = 0.0f;
  float value_pos = 123.456f;
  float value_neg = -123.456f;

  std::vector<std::uint8_t> blob = sdo::serialize<float>(value_zero);
  auto result_zero = sdo::deserialize<float>(blob);
  ASSERT_EQ(result_zero, value_zero);

  blob = sdo::serialize<float>(value_pos);
  auto result_pos = sdo::deserialize<float>(blob);
  ASSERT_EQ(result_pos, value_pos);

  blob = sdo::serialize<float>(value_neg);
  auto result_neg = sdo::deserialize<float>(blob);
  ASSERT_EQ(result_neg, value_neg);
}

TEST(SDOSerializationTest, Double)
{
  double value_zero = 0.0;
  double value_pos = 123.456;
  double value_neg = -123.456;

  std::vector<std::uint8_t> blob = sdo::serialize<double>(value_zero);
  auto result_zero = sdo::deserialize<double>(blob);
  ASSERT_EQ(result_zero, value_zero);

  blob = sdo::serialize<double>(value_pos);
  auto result_pos = sdo::deserialize<double>(blob);
  ASSERT_EQ(result_pos, value_pos);

  blob = sdo::serialize<double>(value_neg);
  auto result_neg = sdo::deserialize<double>(blob);
  ASSERT_EQ(result_neg, value_neg);
}

TEST(SDOSerializationTest, TimeOfDay)
{
  sdo::TimeOfDay value{12345678, 42};

  std::vector<std::uint8_t> blob = sdo::serialize<sdo::TimeOfDay>(value);
  auto result = sdo::deserialize<sdo::TimeOfDay>(blob);
  ASSERT_EQ(result.ms_since_midnight, value.ms_since_midnight);
  ASSERT_EQ(result.d_since_1984_01_01, value.d_since_1984_01_01);
}

TEST(SDOSerializationTest, TimeDifference)
{
  sdo::TimeDifference value{12345678, 42};

  std::vector<std::uint8_t> blob = sdo::serialize<sdo::TimeDifference>(value);
  auto result = sdo::deserialize<sdo::TimeDifference>(blob);
  ASSERT_EQ(result.ms, value.ms);
  ASSERT_EQ(result.d, value.d);
}

TEST(SDOSerializationTest, Domain)
{
  std::vector<std::uint8_t> value = {0, 1, 2, 3};

  std::vector<std::uint8_t> blob = sdo::serialize<std::vector<std::uint8_t>>(value);
  auto result = sdo::deserialize<std::vector<std::uint8_t>>(blob);
  ASSERT_EQ(result, value);
}

TEST(SDOSerializationTest, Int24)
{
  std::int32_t value_zero = 0;
  sdo::Int24 value_max{(std::int64_t{1} << 23) - 1};
  sdo::Int24 value_min{-(std::int64_t{1} << 23)};

  std::vector<std::uint8_t> blob = sdo::serialize<sdo::INTEGER24>(value_zero);
  auto result_zero = sdo::deserialize<sdo::INTEGER24>(blob);
  ASSERT_EQ(result_zero, value_zero);

  blob = sdo::serialize<sdo::Int24>(value_max);
  auto result_max = sdo::deserialize<sdo::Int24>(blob);
  ASSERT_EQ(result_max.value, value_max.value);

  blob = sdo::serialize<sdo::Int24>(value_min);
  auto result_min = sdo::deserialize<sdo::Int24>(blob);
  ASSERT_EQ(result_min.value, value_min.value);
}

TEST(SDOSerializationTest, Int40)
{
  sdo::Int40 value_zero{0};
  sdo::Int40 value_max{(std::int64_t{1} << 39) - 1};
  sdo::Int40 value_min{-(std::int64_t{1} << 39)};

  std::vector<std::uint8_t> blob = sdo::serialize<sdo::Int40>(value_zero);
  auto result_zero = sdo::deserialize<sdo::Int40>(blob);
  ASSERT_EQ(result_zero.value, value_zero.value);

  blob = sdo::serialize<sdo::Int40>(value_max);
  auto result_max = sdo::deserialize<sdo::Int40>(blob);
  ASSERT_EQ(result_max.value, value_max.value);

  blob = sdo::serialize<sdo::Int40>(value_min);
  auto result_min = sdo::deserialize<sdo::Int40>(blob);
  ASSERT_EQ(result_min.value, value_min.value);
}

TEST(SDOSerializationTest, Int48)
{
  sdo::Int48 value_zero{0};
  sdo::Int48 value_max{(std::int64_t{1} << 47) - 1};
  sdo::Int48 value_min{-(std::int64_t{1} << 47)};

  std::vector<std::uint8_t> blob = sdo::serialize<sdo::Int48>(value_zero);
  auto result_zero = sdo::deserialize<sdo::Int48>(blob);
  ASSERT_EQ(result_zero.value, value_zero.value);

  blob = sdo::serialize<sdo::Int48>(value_max);
  auto result_max = sdo::deserialize<sdo::Int48>(blob);
  ASSERT_EQ(result_max.value, value_max.value);

  blob = sdo::serialize<sdo::Int48>(value_min);
  auto result_min = sdo::deserialize<sdo::Int48>(blob);
  ASSERT_EQ(result_min.value, value_min.value);
}

TEST(SDOSerializationTest, Int56)
{
  sdo::Int56 value_zero{0};
  sdo::Int56 value_max{(std::int64_t{1} << 55) - 1};
  sdo::Int56 value_min{-(std::int64_t{1} << 55)};

  std::vector<std::uint8_t> blob = sdo::serialize<sdo::Int56>(value_zero);
  auto result_zero = sdo::deserialize<sdo::Int56>(blob);
  ASSERT_EQ(result_zero.value, value_zero.value);

  blob = sdo::serialize<sdo::Int56>(value_max);
  auto result_max = sdo::deserialize<sdo::Int56>(blob);
  ASSERT_EQ(result_max.value, value_max.value);

  blob = sdo::serialize<sdo::Int56>(value_min);
  auto result_min = sdo::deserialize<sdo::Int56>(blob);
  ASSERT_EQ(result_min.value, value_min.value);
}

TEST(SDOSerializationTest, Int64)
{
  std::int64_t value_zero = 0;
  std::int64_t value_max = std::numeric_limits<std::int64_t>::max();
  std::int64_t value_min = std::numeric_limits<std::int64_t>::min();

  std::vector<std::uint8_t> blob = sdo::serialize<std::int64_t>(value_zero);
  auto result_zero = sdo::deserialize<std::int64_t>(blob);
  ASSERT_EQ(result_zero, value_zero);

  blob = sdo::serialize<std::int64_t>(value_max);
  auto result_max = sdo::deserialize<std::int64_t>(blob);
  ASSERT_EQ(result_max, value_max);

  blob = sdo::serialize<std::int64_t>(value_min);
  auto result_min = sdo::deserialize<std::int64_t>(blob);
  ASSERT_EQ(result_min, value_min);
}

TEST(SDOSerializationTest, uInt24)
{
  sdo::UInt24 value_zero{0};
  sdo::UInt24 value_max{(std::uint64_t{1} << 23) - 1};

  std::vector<std::uint8_t> blob = sdo::serialize<sdo::UInt24>(value_zero);
  auto result_zero = sdo::deserialize<sdo::UInt24>(blob);
  ASSERT_EQ(result_zero.value, value_zero.value);

  blob = sdo::serialize<sdo::UInt24>(value_max);
  auto result_max = sdo::deserialize<sdo::UInt24>(blob);
  ASSERT_EQ(result_max.value, value_max.value);
}

TEST(SDOSerializationTest, uInt40)
{
  sdo::UInt40 value_zero{0};
  sdo::UInt40 value_max{(std::uint64_t{1} << 40) - 1};

  std::vector<std::uint8_t> blob = sdo::serialize<sdo::UInt40>(value_zero);
  auto result_zero = sdo::deserialize<sdo::UInt40>(blob);
  ASSERT_EQ(result_zero.value, value_zero.value);

  blob = sdo::serialize<sdo::UInt40>(value_max);
  auto result_max = sdo::deserialize<sdo::UInt40>(blob);
  ASSERT_EQ(result_max.value, value_max.value);
}

TEST(SDOSerializationTest, uInt48)
{
  sdo::UInt48 value_zero{0};
  sdo::UInt48 value_max{(std::uint64_t{1} << 48) - 1};

  std::vector<std::uint8_t> blob = sdo::serialize<sdo::UInt48>(value_zero);
  auto result_zero = sdo::deserialize<sdo::UInt48>(blob);
  ASSERT_EQ(result_zero.value, value_zero.value);

  blob = sdo::serialize<sdo::UInt48>(value_max);
  auto result_max = sdo::deserialize<sdo::UInt48>(blob);
  ASSERT_EQ(result_max.value, value_max.value);
}

TEST(SDOSerializationTest, uInt56)
{
  sdo::UInt56 value_zero{0};
  sdo::UInt56 value_max{(std::uint64_t{1} << 56) - 1};

  std::vector<std::uint8_t> blob = sdo::serialize<sdo::UInt56>(value_zero);
  auto result_zero = sdo::deserialize<sdo::UInt56>(blob);
  ASSERT_EQ(result_zero.value, value_zero.value);

  blob = sdo::serialize<sdo::UInt56>(value_max);
  auto result_max = sdo::deserialize<sdo::UInt56>(blob);
  ASSERT_EQ(result_max.value, value_max.value);
}

TEST(SDOSerializationTest, uInt64)
{
  std::uint64_t value_zero = 0;
  std::uint64_t value_max = std::numeric_limits<std::uint64_t>::max();

  std::vector<std::uint8_t> blob = sdo::serialize<std::uint64_t>(value_zero);
  auto result_zero = sdo::deserialize<std::uint64_t>(blob);
  ASSERT_EQ(result_zero, value_zero);

  blob = sdo::serialize<std::uint64_t>(value_max);
  auto result_max = sdo::deserialize<std::uint64_t>(blob);
  ASSERT_EQ(result_max, value_max);
}

TEST(SDOSerializationTest, Guid)
{
  sdo::Guid value{{
       1,  2,  3,  4,
       5,  6,  7,  8,
       9, 10, 11, 12,
      13, 14, 15, 16
  }};

  std::vector<std::uint8_t> blob = sdo::serialize<sdo::Guid>(value);
  auto result = sdo::deserialize<sdo::Guid>(blob);
  ASSERT_EQ(result.bytes, value.bytes);
}

TEST(SDOSerializationTest, String50)
{
  std::string string = "RISE";

  std::vector<std::uint8_t> blob = sdo::serialize<sdo::STRING<50>>(string);

  ASSERT_EQ(blob.size(), 50);

  ASSERT_EQ(blob[0], static_cast<std::uint8_t>('R'));
  ASSERT_EQ(blob[1], static_cast<std::uint8_t>('I'));
  ASSERT_EQ(blob[2], static_cast<std::uint8_t>('S'));
  ASSERT_EQ(blob[3], static_cast<std::uint8_t>('E'));

  for (std::size_t i = string.size(); i < blob.size(); ++i)
  {
    ASSERT_EQ(blob[i], 0);
  }

  std::string result = sdo::deserialize<sdo::STRING<50>>(blob);

  ASSERT_EQ(result, string);
}
