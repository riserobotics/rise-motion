// library assumes little-endian

#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>
#include <array>
#include <cstring>
#include <cstddef>


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

    inline std::uint16_t to_16bit_raw(const std::vector<std::uint8_t>& blob, std::size_t offset = 0)
    {
        std::uint16_t raw =
            static_cast<std::uint16_t>(blob[offset + 0]) |
            static_cast<std::uint16_t>(blob[offset + 1]) << 8;
        
        return raw;
    }

    inline std::uint32_t to_24bit_raw(const std::vector<std::uint8_t>& blob, std::size_t offset = 0)
    {
        std::uint32_t raw =
            static_cast<std::uint32_t>(blob[offset + 0]) |
            static_cast<std::uint32_t>(blob[offset + 1]) << 8 |
            static_cast<std::uint32_t>(blob[offset + 2]) << 16;
        
        return raw;
    }

    inline std::uint32_t to_32bit_raw(const std::vector<std::uint8_t>& blob, std::size_t offset = 0)
    {
        std::uint32_t raw =
            static_cast<std::uint32_t>(blob[offset + 0]) |
            static_cast<std::uint32_t>(blob[offset + 1]) << 8 |
            static_cast<std::uint32_t>(blob[offset + 2]) << 16 |
            static_cast<std::uint32_t>(blob[offset + 3]) << 24;
        
        return raw;
    }

    inline std::uint64_t to_40bit_raw(const std::vector<std::uint8_t>& blob, std::size_t offset = 0)
    {
        std::uint64_t raw =
            static_cast<std::uint64_t>(blob[offset + 0]) |
            static_cast<std::uint64_t>(blob[offset + 1]) << 8 |
            static_cast<std::uint64_t>(blob[offset + 2]) << 16 |
            static_cast<std::uint64_t>(blob[offset + 3]) << 24 |
            static_cast<std::uint64_t>(blob[offset + 4]) << 32;
        
        return raw;
    }

    inline std::uint64_t to_48bit_raw(const std::vector<std::uint8_t>& blob, std::size_t offset = 0)
    {
        std::uint64_t raw =
            static_cast<std::uint64_t>(blob[offset + 0]) |
            static_cast<std::uint64_t>(blob[offset + 1]) << 8 |
            static_cast<std::uint64_t>(blob[offset + 2]) << 16 |
            static_cast<std::uint64_t>(blob[offset + 3]) << 24 |
            static_cast<std::uint64_t>(blob[offset + 4]) << 32 |
            static_cast<std::uint64_t>(blob[offset + 5]) << 40;
        
        return raw;
    }

    inline std::uint64_t to_56bit_raw(const std::vector<std::uint8_t>& blob, std::size_t offset = 0)
    {
        std::uint64_t raw =
            static_cast<std::uint64_t>(blob[offset + 0]) |
            static_cast<std::uint64_t>(blob[offset + 1]) << 8 |
            static_cast<std::uint64_t>(blob[offset + 2]) << 16 |
            static_cast<std::uint64_t>(blob[offset + 3]) << 24 |
            static_cast<std::uint64_t>(blob[offset + 4]) << 32 |
            static_cast<std::uint64_t>(blob[offset + 5]) << 40 |
            static_cast<std::uint64_t>(blob[offset + 6]) << 48;
        
        return raw;
    }

    inline std::uint64_t to_64bit_raw(const std::vector<std::uint8_t>& blob, std::size_t offset = 0)
    {
        std::uint64_t raw =
            static_cast<std::uint64_t>(blob[offset + 0]) |
            static_cast<std::uint64_t>(blob[offset + 1]) << 8 |
            static_cast<std::uint64_t>(blob[offset + 2]) << 16 |
            static_cast<std::uint64_t>(blob[offset + 3]) << 24 |
            static_cast<std::uint64_t>(blob[offset + 4]) << 32 |
            static_cast<std::uint64_t>(blob[offset + 5]) << 40 |
            static_cast<std::uint64_t>(blob[offset + 6]) << 48 |
            static_cast<std::uint64_t>(blob[offset + 7]) << 56;
        
        return raw;
    }


    inline std::vector<std::uint8_t> from_16bit_raw(const std::uint16_t& raw)
    {
        return {
            static_cast<std::uint8_t>(raw & 0xFF),
            static_cast<std::uint8_t>((raw >> 8) & 0xFF)
        };
    }

    inline std::vector<std::uint8_t> from_32bit_raw(const std::uint32_t& raw)
    {
        return {
            static_cast<std::uint8_t>(raw & 0xFF),
            static_cast<std::uint8_t>((raw >> 8) & 0xFF),
            static_cast<std::uint8_t>((raw >> 16) & 0xFF),
            static_cast<std::uint8_t>((raw >> 24) & 0xFF)
        };
    }

    inline std::vector<std::uint8_t> from_64bit_raw(const std::uint64_t& raw)
    {
        return {
            static_cast<std::uint8_t>(raw & 0xFF),
            static_cast<std::uint8_t>((raw >> 8) & 0xFF),
            static_cast<std::uint8_t>((raw >> 16) & 0xFF),
            static_cast<std::uint8_t>((raw >> 24) & 0xFF),
            static_cast<std::uint8_t>((raw >> 32) & 0xFF),
            static_cast<std::uint8_t>((raw >> 40) & 0xFF),
            static_cast<std::uint8_t>((raw >> 48) & 0xFF),
            static_cast<std::uint8_t>((raw >> 56) & 0xFF)
        };
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

        std::int16_t value = static_cast<std::int16_t>(sdo::helpers::to_16bit_raw(blob));

        return value;
    }

    template <> inline std::int32_t deserialize<std::int32_t>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 4, "int32_t");

        std::int32_t value = static_cast<std::int32_t>(sdo::helpers::to_32bit_raw(blob));

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

        return sdo::helpers::to_16bit_raw(blob);
    }

    // use for UNSIGNED32, DWORD, BITARR32
    template <> inline std::uint32_t deserialize<std::uint32_t>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 4, "uint32_t");

        return sdo::helpers::to_32bit_raw(blob);
    }

    // Floating Point

    template <> inline float deserialize<float>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 4, "float");

        std::uint32_t raw = sdo::helpers::to_32bit_raw(blob);

        float value;
        std::memcpy(&value, &raw, sizeof(value));

        return value;
    }

    template <> inline double deserialize<double>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 8, "double");

        std::uint64_t raw = sdo::helpers::to_64bit_raw(blob);

        double value;
        std::memcpy(&value, &raw, sizeof(value));

        return value;
    }

    // Time

    template <> inline TimeOfDay deserialize<TimeOfDay>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 6, "TimeOfDay");

        TimeOfDay value{};

        value.ms_since_midnight = sdo::helpers::to_32bit_raw(blob);
        value.d_since_1984_01_01 = sdo::helpers::to_16bit_raw(blob, 4);

        return value;
    }

    template <> inline TimeDifference deserialize<TimeDifference>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 6, "TimeDifference");

        TimeDifference value{};

        // the upper 4 bits of ms are reserved
        value.ms = sdo::helpers::to_32bit_raw(blob) & 0x0FFFFFFF;

        value.d = sdo::helpers::to_16bit_raw(blob, 4);

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

        std::uint32_t raw = sdo::helpers::to_24bit_raw(blob);

        if (raw & 0x00800000){
            raw |= 0xFF000000;
        }

        return Int24{static_cast<std::int32_t>(raw)};
    }

    template <> inline Int40 deserialize<Int40>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 5, "Int40");

        std::uint64_t raw = sdo::helpers::to_40bit_raw(blob);

        if (raw & 0x0000008000000000ULL)
        {
            raw |= 0xFFFFFF0000000000ULL;
        }

        return Int40{static_cast<std::int64_t>(raw)};
    }

    template <> inline Int48 deserialize<Int48>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 6, "Int48");

        std::uint64_t raw = sdo::helpers::to_48bit_raw(blob);

        if (raw & 0x0000800000000000ULL)
        {
            raw |= 0xFFFF000000000000ULL;
        }

        return Int48{static_cast<std::int64_t>(raw)};
    }

    template <> inline Int56 deserialize<Int56>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 7, "Int56");

        std::uint64_t raw = sdo::helpers::to_56bit_raw(blob);

        if (raw & 0x0080000000000000ULL)
        {
            raw |= 0xFF00000000000000ULL;
        }

        return Int56{static_cast<std::int64_t>(raw)};
    }

    template <> inline std::int64_t deserialize<std::int64_t>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 8, "int64_t");

        return static_cast<std::int64_t>(sdo::helpers::to_64bit_raw(blob));
    }

    // Extended Unsigned Integer

    template <> inline UInt24 deserialize<UInt24>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 3, "UInt24");

        return UInt24{sdo::helpers::to_24bit_raw(blob)};
    }

    template <> inline UInt40 deserialize<UInt40>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 5, "UInt40");

        return UInt40{sdo::helpers::to_40bit_raw(blob)};
    }

    template <> inline UInt48 deserialize<UInt48>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 6, "UInt48");

        return UInt48{sdo::helpers::to_48bit_raw(blob)};
    }

    template <> inline UInt56 deserialize<UInt56>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 7, "UInt56");

        return UInt56{sdo::helpers::to_56bit_raw(blob)};
    }

    template <> inline std::uint64_t deserialize<std::uint64_t>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 8, "uint64_t");

        return sdo::helpers::to_64bit_raw(blob);
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
        return sdo::helpers::from_16bit_raw(static_cast<std::uint16_t>(value));
    }

    template <> inline std::vector<std::uint8_t> serialize<std::int32_t>(const std::int32_t& value)
    {
        return sdo::helpers::from_32bit_raw(static_cast<std::uint32_t>(value));
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
        return sdo::helpers::from_16bit_raw(value);
    }

    // use for UNSIGNED32, DWORD, BITARR32

    template <> inline std::vector<std::uint8_t> serialize<std::uint32_t>(const std::uint32_t& value)
    {
        return sdo::helpers::from_32bit_raw(value);
    }

    // Floating Point

    template <> inline std::vector<std::uint8_t> serialize<float>(const float& value)
    {
        std::uint32_t raw;
        std::memcpy(&raw, &value, sizeof(raw));

        return sdo::helpers::from_32bit_raw(raw);
    }

    template <> inline std::vector<std::uint8_t> serialize<double>(const double& value)
    {
        std::uint64_t raw;
        std::memcpy(&raw, &value, sizeof(raw));

        return sdo::helpers::from_64bit_raw(raw);
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
}
