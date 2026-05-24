// library assumes little-endian

#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>
#include <array>
#include <cstring>
#include <cstddef>
#include <cmath>


namespace sdo::helpers
{
    template <typename> inline constexpr bool always_false = false;

    inline void check_size(const std::vector<std::uint8_t>& blob, std::size_t expectedSize, const char* type)
    {
        if (blob.size() != expectedSize){
            throw std::invalid_argument(std::string("deserialize<") + type +
                ">: expected " + std::to_string(expectedSize) + " bytes, got " + std::to_string(blob.size()));
        }
    }

    template <typename T> inline T to_raw(
        const std::vector<std::uint8_t>& blob, std::size_t numBytes, std::size_t offset = 0)
    {
        static_assert(std::is_unsigned_v<T>, "to_raw<T>: T must be unsigned");
        static_assert(sizeof(T) <= sizeof(std::uint64_t), "to_raw<T>: T can be max uint64_t");

        if (numBytes > sizeof(T))
        {
            throw std::invalid_argument("to_raw<T>: numBytes does not fit T");
        }

        if (offset + numBytes > blob.size())
        {
            throw std::out_of_range("to_raw<T>: blob has less bytes than numBytes");
        }

        T raw = 0;

        for (std::size_t i = 0; i < numBytes; ++i)
        {
            raw |= static_cast<T>(blob[offset + i]) << (8 * i);
        }

        return raw;
    }

    template <typename T> inline std::vector<std::uint8_t> from_raw(const T& raw, std::size_t numBytes)
    {
        static_assert(std::is_unsigned_v<T>, "from_raw<T>: T must be unsigned");

        if (numBytes > sizeof(T))
        {
            throw std::invalid_argument("from_raw<T>: numBytes does not fit T");
        }

        std::vector<std::uint8_t> blob;
        blob.reserve(numBytes);

        for (std::size_t i = 0; i < numBytes; ++i)
        {
            blob.push_back(static_cast<std::uint8_t>((raw >> (8 * i)) & 0xFF));
        }

        return blob;
    }
}

namespace sdo
{
    // as equivalent of the EtherCAT TIME_OF_DAY data type
    struct TimeOfDay
    {
        std::uint32_t ms_since_midnight;
        std::uint16_t d_since_1984_01_01;
    };

    // as equivalent of the EtherCAT TIME_DIFFERENCE data type
    struct TimeDifference
    {
        std::uint32_t ms;
        std::uint16_t d;
    };

    struct Int24
    {
        std::int32_t value;
    };

    struct Int40
    {
        std::int64_t value;
    };

    struct Int48
    {
        std::int64_t value;
    };

    struct Int56
    {
        std::int64_t value;
    };

    struct UInt24
    {
        std::uint32_t value;
    };

    struct UInt40
    {
        std::uint64_t value;
    };

    struct UInt48
    {
        std::uint64_t value;
    };

    struct UInt56
    {
        std::uint64_t value;
    };

    struct Guid
    {
        std::array<std::uint8_t, 16> bytes;
    };


    template <typename T> T deserialize(const std::vector<std::uint8_t>&)
    {
        static_assert(sdo::helpers::always_false<T>, "deserialize<T>: unsupported type");
    }

    // Boolean

    template <> inline bool deserialize<bool>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 1, "bool");

        return blob[0] != 0;
    }

    // Signed Integer

    template <> inline std::int8_t deserialize<std::int8_t>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 1, "int8_t");

        return static_cast<std::int8_t>(blob[0]);
    }

    template <> inline std::int16_t deserialize<std::int16_t>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 2, "int16_t");

        std::int16_t value = static_cast<std::int16_t>(sdo::helpers::to_raw<std::uint16_t>(blob, 2));

        return value;
    }

    template <> inline std::int32_t deserialize<std::int32_t>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 4, "int32_t");

        std::int32_t value = static_cast<std::int32_t>(sdo::helpers::to_raw<std::uint32_t>(blob, 4));

        return value;
    }

    // Unsigned Integer / raw data / bit arrays / bit strings

    // use for UNSIGNED8, BYTE, BITARR8, BIT1-BIT8
    template <> inline std::uint8_t deserialize<std::uint8_t>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 1, "uint8_t");

        return static_cast<std::uint8_t>(blob[0]);
    }

    // use for UNSIGNED16, WORD, BITARR16, BIT9-BIT16
    template <> inline std::uint16_t deserialize<std::uint16_t>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 2, "uint16_t");

        return sdo::helpers::to_raw<std::uint16_t>(blob, 2);
    }

    // use for UNSIGNED32, DWORD, BITARR32
    template <> inline std::uint32_t deserialize<std::uint32_t>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 4, "uint32_t");

        return sdo::helpers::to_raw<std::uint32_t>(blob, 4);
    }

    // Floating Point

    template <> inline float deserialize<float>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 4, "float");

        std::uint32_t raw = sdo::helpers::to_raw<std::uint32_t>(blob, 4);

        float value;
        std::memcpy(&value, &raw, sizeof(value));

        return value;
    }

    template <> inline double deserialize<double>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 8, "double");

        std::uint64_t raw = sdo::helpers::to_raw<std::uint64_t>(blob, 8);

        double value;
        std::memcpy(&value, &raw, sizeof(value));

        return value;
    }

    // Time

    template <> inline TimeOfDay deserialize<TimeOfDay>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 6, "TimeOfDay");

        TimeOfDay value{};

        value.ms_since_midnight = sdo::helpers::to_raw<std::uint32_t>(blob, 4);
        value.d_since_1984_01_01 = sdo::helpers::to_raw<std::uint16_t>(blob, 2, 4);

        return value;
    }

    template <> inline TimeDifference deserialize<TimeDifference>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 6, "TimeDifference");

        TimeDifference value{};

        // the upper 4 bits of ms are reserved
        value.ms = sdo::helpers::to_raw<std::uint32_t>(blob, 4) & 0x0FFFFFFF;

        value.d = sdo::helpers::to_raw<std::uint16_t>(blob, 2, 4);

        return value;
    }

    // Domain (returns raw blob as equivalent to the EtherCAT Domain data type)

    template <> inline std::vector<std::uint8_t> deserialize<std::vector<std::uint8_t>>(
        const std::vector<std::uint8_t>& blob)
    {
        return blob;
    }

    // Extended Signed Integer

    template <> inline Int24 deserialize<Int24>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 3, "Int24");

        std::uint32_t raw = sdo::helpers::to_raw<std::uint32_t>(blob, 3);

        if (raw & 0x00800000){
            raw |= 0xFF000000;
        }

        return Int24{static_cast<std::int32_t>(raw)};
    }

    template <> inline Int40 deserialize<Int40>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 5, "Int40");

        std::uint64_t raw = sdo::helpers::to_raw<std::uint64_t>(blob, 5);

        if (raw & 0x0000008000000000ULL)
        {
            raw |= 0xFFFFFF0000000000ULL;
        }

        return Int40{static_cast<std::int64_t>(raw)};
    }

    template <> inline Int48 deserialize<Int48>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 6, "Int48");

        std::uint64_t raw = sdo::helpers::to_raw<std::uint64_t>(blob, 6);

        if (raw & 0x0000800000000000ULL)
        {
            raw |= 0xFFFF000000000000ULL;
        }

        return Int48{static_cast<std::int64_t>(raw)};
    }

    template <> inline Int56 deserialize<Int56>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 7, "Int56");

        std::uint64_t raw = sdo::helpers::to_raw<std::uint64_t>(blob, 7);

        if (raw & 0x0080000000000000ULL)
        {
            raw |= 0xFF00000000000000ULL;
        }

        return Int56{static_cast<std::int64_t>(raw)};
    }

    template <> inline std::int64_t deserialize<std::int64_t>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 8, "int64_t");

        return static_cast<std::int64_t>(sdo::helpers::to_raw<std::uint64_t>(blob, 8));
    }

    // Extended Unsigned Integer

    template <> inline UInt24 deserialize<UInt24>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 3, "UInt24");

        return UInt24{sdo::helpers::to_raw<std::uint32_t>(blob, 3)};
    }

    template <> inline UInt40 deserialize<UInt40>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 5, "UInt40");

        return UInt40{sdo::helpers::to_raw<std::uint64_t>(blob, 5)};
    }

    template <> inline UInt48 deserialize<UInt48>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 6, "UInt48");

        return UInt48{sdo::helpers::to_raw<std::uint64_t>(blob, 6)};
    }

    template <> inline UInt56 deserialize<UInt56>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 7, "UInt56");

        return UInt56{sdo::helpers::to_raw<std::uint64_t>(blob, 7)};
    }

    template <> inline std::uint64_t deserialize<std::uint64_t>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 8, "uint64_t");

        return sdo::helpers::to_raw<std::uint64_t>(blob, 8);
    }

    // GUID

    template <> inline Guid deserialize<Guid>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 16, "Guid");

        Guid value{};

        for (std::size_t i = 0; i < value.bytes.size(); ++i)
        {
            value.bytes[i] = blob[i];
        }

        return value;
    }



    template <typename T> std::vector<std::uint8_t> serialize(const T& value)
    {
        static_assert(sdo::helpers::always_false<T>, "serialize<T>: unsupported type");
        return{};
    }

    // Boolean

    template <> inline std::vector<std::uint8_t> serialize<bool>(const bool& value)
    {
        return {static_cast<std::uint8_t>(value ? 1 : 0)};
    }

    // Signed Integer

    template <> inline std::vector<std::uint8_t> serialize<std::int8_t>(const std::int8_t& value)
    {
        return {static_cast<std::uint8_t>(value)};
    }

    template <> inline std::vector<std::uint8_t> serialize<std::int16_t>(const std::int16_t& value)
    {
        return sdo::helpers::from_raw<std::uint16_t>(static_cast<std::uint16_t>(value), 2);
    }

    template <> inline std::vector<std::uint8_t> serialize<std::int32_t>(const std::int32_t& value)
    {
        return sdo::helpers::from_raw<std::uint32_t>(static_cast<std::uint32_t>(value), 4);
    }

    // Unsigned Integer / raw data / bit arrays / bit strings

    // use for UNSIGNED8, BYTE, BITARR8, BIT1-BIT8
    template <> inline std::vector<std::uint8_t> serialize<std::uint8_t>(const std::uint8_t& value)
    {
        return { value };
    }

    // use for UNSIGNED16, WORD, BITARR16, BIT9-BIT16
    template <> inline std::vector<std::uint8_t> serialize<std::uint16_t>(const std::uint16_t& value)
    {
        return sdo::helpers::from_raw<std::uint16_t>(value, 2);
    }

    // use for UNSIGNED32, DWORD, BITARR32

    template <> inline std::vector<std::uint8_t> serialize<std::uint32_t>(const std::uint32_t& value)
    {
        return sdo::helpers::from_raw<std::uint32_t>(value, 4);
    }

    // Floating Point

    template <> inline std::vector<std::uint8_t> serialize<float>(const float& value)
    {
        std::uint32_t raw;
        std::memcpy(&raw, &value, sizeof(raw));

        return sdo::helpers::from_raw<std::uint32_t>(raw, 4);
    }

    template <> inline std::vector<std::uint8_t> serialize<double>(const double& value)
    {
        std::uint64_t raw;
        std::memcpy(&raw, &value, sizeof(raw));

        return sdo::helpers::from_raw<std::uint64_t>(raw, 8);
    }

    // Time

    template <> inline std::vector<std::uint8_t> serialize<TimeOfDay>(const TimeOfDay& value)
    {
        return {
            static_cast<std::uint8_t>(value.ms_since_midnight & 0xFF),
            static_cast<std::uint8_t>((value.ms_since_midnight >> 8) & 0xFF),
            static_cast<std::uint8_t>((value.ms_since_midnight >> 16) & 0xFF),
            static_cast<std::uint8_t>((value.ms_since_midnight >> 24) & 0xFF),

            static_cast<std::uint8_t>(value.d_since_1984_01_01 & 0xFF),
            static_cast<std::uint8_t>((value.d_since_1984_01_01 >> 8) & 0xFF)
        };
    }

    template <> inline std::vector<std::uint8_t> serialize<TimeDifference>(const TimeDifference& value)
    {
        if (value.ms > 0x0FFFFFFF)
        {
            throw std::out_of_range("serialize<TimeDifference>: TimeDifference.ms exceeds 28 bit range");
        }

        return {
            static_cast<std::uint8_t>(value.ms & 0xFF),
            static_cast<std::uint8_t>((value.ms >> 8) & 0xFF),
            static_cast<std::uint8_t>((value.ms >> 16) & 0xFF),
            static_cast<std::uint8_t>((value.ms >> 24) & 0xFF),

            static_cast<std::uint8_t>(value.d & 0xFF),
            static_cast<std::uint8_t>((value.d >> 8) & 0xFF)
        };
    }

    // Domain (returns raw blob as equivalent to the EtherCAT Domain data type)

    template <> inline std::vector<std::uint8_t> serialize<std::vector<std::uint8_t>>(
        const std::vector<std::uint8_t>& value)
    {
        return value;
    }

    // Extended Signed Integer

    template <> inline std::vector<std::uint8_t> serialize<Int24>(const Int24& value)
    {
        if (value.value < -pow(2, 23) || value.value > pow(2, 23)-1)
        {
            throw std::out_of_range("Int24 value exceeds 24 bit signed range");
        }

        const auto raw = static_cast<std::uint32_t>(value.value);

        return sdo::helpers::from_raw<std::uint32_t>(raw, 3);
    }

    template <> inline std::vector<std::uint8_t> serialize<Int40>(const Int40& value)
    {
        if (value.value < -pow(2, 39) || value.value > pow(2, 39)-1)
        {
            throw std::out_of_range("Int40 value exceeds 40 bit signed range");
        }

        const auto raw = static_cast<std::uint64_t>(value.value);

        return sdo::helpers::from_raw<std::uint64_t>(raw, 5);
    }

    template <> inline std::vector<std::uint8_t> serialize<Int48>(const Int48& value)
    {
        if (value.value < -pow(2, 47) || value.value > pow(2, 47)-1)
        {
            throw std::out_of_range("Int48 value exceeds 48 bit signed range");
        }

        const auto raw = static_cast<std::uint64_t>(value.value);

        return sdo::helpers::from_raw<std::uint64_t>(raw, 6);
    }

    template <> inline std::vector<std::uint8_t> serialize<Int56>(const Int56& value)
    {
        if (value.value < -pow(2, 55) || value.value > pow(2, 55)-1)
        {
            throw std::out_of_range("Int56 value exceeds 56 bit signed range");
        }

        const auto raw = static_cast<std::uint64_t>(value.value);

        return sdo::helpers::from_raw<std::uint64_t>(raw, 7);
    }

    template <> inline std::vector<std::uint8_t> serialize<int64_t>(const int64_t& value)
    {
        return sdo::helpers::from_raw<std::uint64_t>(static_cast<std::uint64_t>(value), 8);
    }

    // Extended unsigned Integer

    template <> inline std::vector<std::uint8_t> serialize<UInt24>(const UInt24& value)
    {
        if (value.value > pow(2, 24)-1)
        {
            throw std::out_of_range("UInt24 value exceeds 24 bit unsigned range");
        }

        return sdo::helpers::from_raw<std::uint32_t>(value.value, 3);
    }

    template <> inline std::vector<std::uint8_t> serialize<UInt40>(const UInt40& value)
    {
        if (value.value > pow(2, 40)-1)
        {
            throw std::out_of_range("UInt40 value exceeds 40 bit unsigned range");
        }

        return sdo::helpers::from_raw<std::uint64_t>(value.value, 5);
    }

    template <> inline std::vector<std::uint8_t> serialize<UInt48>(const UInt48& value)
    {
        if (value.value > pow(2, 48)-1)
        {
            throw std::out_of_range("UInt48 value exceeds 48 bit unsigned range");
        }

        return sdo::helpers::from_raw<std::uint64_t>(value.value, 6);
    }

    template <> inline std::vector<std::uint8_t> serialize<UInt56>(const UInt56& value)
    {
        if (value.value > pow(2, 56)-1)
        {
            throw std::out_of_range("UInt56 value exceeds 56 bit unsigned range");
        }

        return sdo::helpers::from_raw<std::uint64_t>(value.value, 7);
    }

    template <> inline std::vector<std::uint8_t> serialize<uint64_t>(const uint64_t& value)
    {
        return sdo::helpers::from_raw<std::uint64_t>(value, 8);
    }

    // GUID

    template <> inline std::vector<std::uint8_t> serialize<Guid>(const Guid& value)
    {
        return std::vector<std::uint8_t>(value.bytes.begin(), value.bytes.end());
    }
}
