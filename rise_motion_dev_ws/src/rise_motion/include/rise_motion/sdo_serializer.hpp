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

        Int24(std::int32_t v) : value(v)
        {
            if (v < -pow(2, 23) || v > pow(2, 23)-1){
                throw std::out_of_range("Can not convert int32_t value exceeding 24 bit signed range into Int24");
            }
        }

        operator std::int32_t() const
        {
            return value;
        }
    };

    struct Int40
    {
        std::int64_t value;

        Int40(std::int64_t v) : value(v)
        {
            if (v < -pow(2, 39) || v > pow(2, 39)-1){
                throw std::out_of_range("Can not convert int64_t value exceeding 40 bit signed range into Int40");
            }
        }

        operator std::int64_t() const
        {
            return value;
        }
    };

    struct Int48
    {
        std::int64_t value;

        Int48(std::int64_t v) : value(v)
        {
            if (v < -pow(2, 47) || v > pow(2, 47)-1){
                throw std::out_of_range("Can not convert int64_t value exceeding 48 bit signed range into Int48");
            }
        }

        operator std::int64_t() const
        {
            return value;
        }
    };

    struct Int56
    {
        std::int64_t value;

        Int56(std::int64_t v) : value(v)
        {
            if (v < -pow(2, 55) || v > pow(2, 55)-1){
                throw std::out_of_range("Can not convert int64_t value exceeding 56 bit signed range into Int56");
            }
        }

        operator std::int64_t() const
        {
            return value;
        }
    };

    struct UInt24
    {
        std::uint32_t value;

        UInt24(std::uint32_t v) : value(v)
        {
            if (v > pow(2, 24)-1){
                throw std::out_of_range("Can not convert uint32_t value exceeding 24 bit unsigned range into UInt24");
            }
        }

        operator std::uint32_t() const
        {
            return value;
        }
    };

    struct UInt40
    {
        std::uint64_t value;

        UInt40(std::uint64_t v) : value(v)
        {
            if (v > pow(2, 40)-1){
                throw std::out_of_range("Can not convert uint64_t value exceeding 40 bit unsigned range into UInt40");
            }
        }

        operator std::uint64_t() const
        {
            return value;
        }
    };

    struct UInt48
    {
        std::uint64_t value;

        UInt48(std::uint64_t v) : value(v)
        {
            if (v > pow(2, 48)-1){
                throw std::out_of_range("Can not convert uint64_t value exceeding 48 bit unsigned range into UInt48");
            }
        }

        operator std::uint64_t() const
        {
            return value;
        }
    };

    struct UInt56
    {
        std::uint64_t value;

        UInt56(std::uint64_t v) : value(v)
        {
            if (v > pow(2, 56)-1){
                throw std::out_of_range("Can not convert uint64_t value exceeding 56 bit unsigned range into UInt56");
            }
        }

        operator std::uint64_t() const
        {
            return value;
        }
    };

    struct Guid
    {
        std::array<std::uint8_t, 16> bytes;

        Guid(std::array<std::uint8_t, 16> b) : bytes(b){};
        Guid() = default;

        operator std::array<std::uint8_t, 16>() const
        {
            return bytes;
        }
    };

    template <std::size_t T> struct STRING
    {
        std::string value;

        STRING() = default;

        STRING(std::string v) : value(std::move(v))
        {
            if (value.size() > T){
                throw std::out_of_range("STRING<T>: string is too long to be coverted to STRING<T> of size T");
            }
        };

        operator std::string() const
        {
            return value;
        }
    };


    // IEC 61131-3 data types
    using BOOL  = bool;

    using SINT  = std::int8_t;
    using INT   = std::int16_t;
    using DINT  = std::int32_t;
    using LINT  = std::int64_t;

    using USINT = std::uint8_t;
    using UINT  = std::uint16_t;
    using UDINT = std::uint32_t;
    using ULINT = std::uint64_t;

    using REAL  = float;
    using LREAL = double;

    using BYTE  = std::uint8_t;
    using WORD  = std::uint16_t;
    using DWORD = std::uint32_t;
    using LWORD = std::uint64_t;

    using DATE  = sdo::TimeOfDay;
    using TIME  = sdo::TimeDifference;

    // EtherCAT data types
    using BOOLEAN = bool;

    using INTEGER8  = std::int8_t;
    using INTEGER16 = std::int16_t;
    using INTEGER24 = sdo::Int24;
    using INTEGER32 = std::int32_t;
    using INTEGER40 = sdo::Int40;
    using INTEGER48 = sdo::Int48;
    using INTEGER56 = sdo::Int56;
    using INTEGER64 = std::int64_t;

    using UNSIGNED8  = std::uint8_t;
    using UNSIGNED16 = std::uint16_t;
    using UNSIGNED24 = sdo::UInt24;
    using UNSIGNED32 = std::uint32_t;
    using UNSIGNED40 = sdo::UInt40;
    using UNSIGNED48 = sdo::UInt48;
    using UNSIGNED56 = sdo::UInt56;
    using UNSIGNED64 = std::uint64_t;

    using REAL32 = float;
    using REAL64 = double;

    using TIME_OF_DAY     = sdo::TimeOfDay;
    using TIME_DIFFERENCE = sdo::TimeDifference;

    using GUID   = sdo::Guid;
    using DOMAIN = std::vector<std::uint8_t>;
}


namespace sdo::helpers
{
    template <typename> inline constexpr bool always_false = false;

    template <typename T> struct is_string_type : std::false_type {};
    template <std::size_t T> struct is_string_type<sdo::STRING<T>> : std::true_type {};

    template <typename T> struct string_size;
    template <std::size_t T> struct string_size<sdo::STRING<T>>
    {
        static constexpr std::size_t size = T;
    };

    inline void check_size(const std::vector<std::uint8_t>& blob, std::size_t expectedSize, const char* type)
    {
        if (blob.size() != expectedSize){
            throw std::invalid_argument(std::string("deserialize<") + type + ">: expected " + 
                std::to_string(expectedSize) + " bytes, got " + std::to_string(blob.size()));
        }
    }

    template <typename T> [[nodiscard]] inline T to_raw(
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

    template <typename T> [[nodiscard]] inline std::vector<std::uint8_t> from_raw(const T& raw, std::size_t numBytes)
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
    // ---DESERIALIZATION---

    template <typename T> T deserialize(const std::vector<std::uint8_t>& blob)
    {
        if constexpr (sdo::helpers::is_string_type<std::remove_cv_t<std::remove_reference_t<T>>>::value){
            
            constexpr std::size_t size = sdo::helpers::string_size<T>::size;

            if (blob.size() > size){
                throw std::invalid_argument("deserialize<STRING<T>>: blob is larger than the expected size T");
            }

            T string{};

            string.value.assign(blob.begin(), blob.end());

            // strip padding zeros
            while (!string.value.empty() && string.value.back() == '\0'){
                string.value.pop_back();
            }

            return string;
        }
        else{
            static_assert(sdo::helpers::always_false<T>, "deserialize<T>: unsupported type");
        }
    }

    // -Boolean-

    template <> [[nodiscard]] inline bool deserialize<bool>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 1, "bool");

        return blob[0] != 0;
    }

    // -Signed Integer-

    template <> [[nodiscard]] inline std::int8_t deserialize<std::int8_t>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 1, "int8_t");

        return static_cast<std::int8_t>(blob[0]);
    }

    template <> [[nodiscard]] inline std::int16_t deserialize<std::int16_t>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 2, "int16_t");

        std::int16_t value = static_cast<std::int16_t>(sdo::helpers::to_raw<std::uint16_t>(blob, 2));

        return value;
    }

    template <> [[nodiscard]] inline std::int32_t deserialize<std::int32_t>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 4, "int32_t");

        std::int32_t value = static_cast<std::int32_t>(sdo::helpers::to_raw<std::uint32_t>(blob, 4));

        return value;
    }

    // -Unsigned Integer / raw data / bit arrays / bit strings-

    // use for UNSIGNED8, BYTE, BITARR8, BIT1-BIT8
    template <> [[nodiscard]] inline std::uint8_t deserialize<std::uint8_t>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 1, "uint8_t");

        return static_cast<std::uint8_t>(blob[0]);
    }

    // use for UNSIGNED16, WORD, BITARR16, BIT9-BIT16
    template <> [[nodiscard]] inline std::uint16_t deserialize<std::uint16_t>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 2, "uint16_t");

        return sdo::helpers::to_raw<std::uint16_t>(blob, 2);
    }

    // use for UNSIGNED32, DWORD, BITARR32
    template <> [[nodiscard]] inline std::uint32_t deserialize<std::uint32_t>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 4, "uint32_t");

        return sdo::helpers::to_raw<std::uint32_t>(blob, 4);
    }

    // -Floating Point-

    template <> [[nodiscard]] inline float deserialize<float>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 4, "float");

        std::uint32_t raw = sdo::helpers::to_raw<std::uint32_t>(blob, 4);

        float value;
        std::memcpy(&value, &raw, sizeof(value));

        return value;
    }

    template <> [[nodiscard]] inline double deserialize<double>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 8, "double");

        std::uint64_t raw = sdo::helpers::to_raw<std::uint64_t>(blob, 8);

        double value;
        std::memcpy(&value, &raw, sizeof(value));

        return value;
    }

    // -Time-

    template <> [[nodiscard]] inline TimeOfDay deserialize<TimeOfDay>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 6, "TimeOfDay");

        TimeOfDay value{};

        value.ms_since_midnight = sdo::helpers::to_raw<std::uint32_t>(blob, 4);
        value.d_since_1984_01_01 = sdo::helpers::to_raw<std::uint16_t>(blob, 2, 4);

        return value;
    }

    template <> [[nodiscard]] inline TimeDifference deserialize<TimeDifference>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 6, "TimeDifference");

        TimeDifference value{};

        // the upper 4 bits of ms are reserved
        value.ms = sdo::helpers::to_raw<std::uint32_t>(blob, 4) & 0x0FFFFFFF;

        value.d = sdo::helpers::to_raw<std::uint16_t>(blob, 2, 4);

        return value;
    }

    // -Domain-
    // (returns raw blob as equivalent to the EtherCAT Domain data type)

    template <> [[nodiscard]] inline std::vector<std::uint8_t> deserialize<std::vector<std::uint8_t>>(
        const std::vector<std::uint8_t>& blob)
    {
        return blob;
    }

    // -Extended Signed Integer-

    template <> [[nodiscard]] inline Int24 deserialize<Int24>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 3, "Int24");

        std::uint32_t raw = sdo::helpers::to_raw<std::uint32_t>(blob, 3);

        if (raw & 0x00800000){
            raw |= 0xFF000000;
        }

        return Int24{static_cast<std::int32_t>(raw)};
    }

    template <> [[nodiscard]] inline Int40 deserialize<Int40>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 5, "Int40");

        std::uint64_t raw = sdo::helpers::to_raw<std::uint64_t>(blob, 5);

        if (raw & 0x0000008000000000ULL)
        {
            raw |= 0xFFFFFF0000000000ULL;
        }

        return Int40{static_cast<std::int64_t>(raw)};
    }

    template <> [[nodiscard]] inline Int48 deserialize<Int48>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 6, "Int48");

        std::uint64_t raw = sdo::helpers::to_raw<std::uint64_t>(blob, 6);

        if (raw & 0x0000800000000000ULL)
        {
            raw |= 0xFFFF000000000000ULL;
        }

        return Int48{static_cast<std::int64_t>(raw)};
    }

    template <> [[nodiscard]] inline Int56 deserialize<Int56>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 7, "Int56");

        std::uint64_t raw = sdo::helpers::to_raw<std::uint64_t>(blob, 7);

        if (raw & 0x0080000000000000ULL)
        {
            raw |= 0xFF00000000000000ULL;
        }

        return Int56{static_cast<std::int64_t>(raw)};
    }

    template <> [[nodiscard]] inline std::int64_t deserialize<std::int64_t>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 8, "int64_t");

        return static_cast<std::int64_t>(sdo::helpers::to_raw<std::uint64_t>(blob, 8));
    }

    // -Extended Unsigned Integer-

    template <> [[nodiscard]] inline UInt24 deserialize<UInt24>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 3, "UInt24");

        return UInt24{sdo::helpers::to_raw<std::uint32_t>(blob, 3)};
    }

    template <> [[nodiscard]] inline UInt40 deserialize<UInt40>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 5, "UInt40");

        return UInt40{sdo::helpers::to_raw<std::uint64_t>(blob, 5)};
    }

    template <> [[nodiscard]] inline UInt48 deserialize<UInt48>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 6, "UInt48");

        return UInt48{sdo::helpers::to_raw<std::uint64_t>(blob, 6)};
    }

    template <> [[nodiscard]] inline UInt56 deserialize<UInt56>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 7, "UInt56");

        return UInt56{sdo::helpers::to_raw<std::uint64_t>(blob, 7)};
    }

    template <> [[nodiscard]] inline std::uint64_t deserialize<std::uint64_t>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 8, "uint64_t");

        return sdo::helpers::to_raw<std::uint64_t>(blob, 8);
    }

    // -GUID-

    template <> [[nodiscard]] inline Guid deserialize<Guid>(const std::vector<std::uint8_t>& blob)
    {
        sdo::helpers::check_size(blob, 16, "Guid");

        Guid value{};

        for (std::size_t i = 0; i < value.bytes.size(); ++i)
        {
            value.bytes[i] = blob[i];
        }

        return value;
    }


    // ---SERIALIZATION---

    template <typename T> std::vector<std::uint8_t> serialize(const T& value)
    {
        if constexpr (sdo::helpers::is_string_type<std::remove_cv_t<std::remove_reference_t<T>>>::value){

            constexpr std::size_t size = sdo::helpers::string_size<T>::size;

            if (value.value.size() > size){
                throw std::out_of_range("serialize<STRING<T>>: string is longer than size T");
            }

            std::vector<std::uint8_t> blob(value.value.begin(), value.value.end());
            blob.resize(size, '\0');

            return blob;
        }
        else{
            static_assert(sdo::helpers::always_false<T>, "serialize<T>: unsupported type");
        }
        return{};
    }

    // -Boolean-

    template <> [[nodiscard]] inline std::vector<std::uint8_t> serialize<bool>(const bool& value)
    {
        return {static_cast<std::uint8_t>(value ? 1 : 0)};
    }

    // -Signed Integer-

    template <> [[nodiscard]] inline std::vector<std::uint8_t> serialize<std::int8_t>(const std::int8_t& value)
    {
        return {static_cast<std::uint8_t>(value)};
    }

    template <> [[nodiscard]] inline std::vector<std::uint8_t> serialize<std::int16_t>(const std::int16_t& value)
    {
        return sdo::helpers::from_raw<std::uint16_t>(static_cast<std::uint16_t>(value), 2);
    }

    template <> [[nodiscard]] inline std::vector<std::uint8_t> serialize<std::int32_t>(const std::int32_t& value)
    {
        return sdo::helpers::from_raw<std::uint32_t>(static_cast<std::uint32_t>(value), 4);
    }

    // -Unsigned Integer / raw data / bit arrays / bit strings-

    // use for UNSIGNED8, BYTE, BITARR8, BIT1-BIT8
    template <> [[nodiscard]] inline std::vector<std::uint8_t> serialize<std::uint8_t>(const std::uint8_t& value)
    {
        return { value };
    }

    // use for UNSIGNED16, WORD, BITARR16, BIT9-BIT16
    template <> [[nodiscard]] inline std::vector<std::uint8_t> serialize<std::uint16_t>(const std::uint16_t& value)
    {
        return sdo::helpers::from_raw<std::uint16_t>(value, 2);
    }

    // use for UNSIGNED32, DWORD, BITARR32

    template <> [[nodiscard]] inline std::vector<std::uint8_t> serialize<std::uint32_t>(const std::uint32_t& value)
    {
        return sdo::helpers::from_raw<std::uint32_t>(value, 4);
    }

    // -Floating Point-

    template <> [[nodiscard]] inline std::vector<std::uint8_t> serialize<float>(const float& value)
    {
        std::uint32_t raw;
        std::memcpy(&raw, &value, sizeof(raw));

        return sdo::helpers::from_raw<std::uint32_t>(raw, 4);
    }

    template <> [[nodiscard]] inline std::vector<std::uint8_t> serialize<double>(const double& value)
    {
        std::uint64_t raw;
        std::memcpy(&raw, &value, sizeof(raw));

        return sdo::helpers::from_raw<std::uint64_t>(raw, 8);
    }

    // -Time-

    template <> [[nodiscard]] inline std::vector<std::uint8_t> serialize<TimeOfDay>(const TimeOfDay& value)
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

    template <> [[nodiscard]] inline std::vector<std::uint8_t> serialize<TimeDifference>(const TimeDifference& value)
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

    // -Domain-
    // (returns raw blob as equivalent to the EtherCAT Domain data type)

    template <> [[nodiscard]] inline std::vector<std::uint8_t> serialize<std::vector<std::uint8_t>>(
        const std::vector<std::uint8_t>& value)
    {
        return value;
    }

    // -Extended Signed Integer-

    template <> [[nodiscard]] inline std::vector<std::uint8_t> serialize<Int24>(const Int24& value)
    {
        if (value.value < -pow(2, 23) || value.value > pow(2, 23)-1){
            throw std::out_of_range("Int24 value exceeds 24 bit signed range");
        }

        const auto raw = static_cast<std::uint32_t>(value.value);

        return sdo::helpers::from_raw<std::uint32_t>(raw, 3);
    }

    template <> [[nodiscard]] inline std::vector<std::uint8_t> serialize<Int40>(const Int40& value)
    {
        if (value.value < -pow(2, 39) || value.value > pow(2, 39)-1){
            throw std::out_of_range("Int40 value exceeds 40 bit signed range");
        }

        const auto raw = static_cast<std::uint64_t>(value.value);

        return sdo::helpers::from_raw<std::uint64_t>(raw, 5);
    }

    template <> [[nodiscard]] inline std::vector<std::uint8_t> serialize<Int48>(const Int48& value)
    {
        if (value.value < -pow(2, 47) || value.value > pow(2, 47)-1){
            throw std::out_of_range("Int48 value exceeds 48 bit signed range");
        }

        const auto raw = static_cast<std::uint64_t>(value.value);

        return sdo::helpers::from_raw<std::uint64_t>(raw, 6);
    }

    template <> [[nodiscard]] inline std::vector<std::uint8_t> serialize<Int56>(const Int56& value)
    {
        if (value.value < -pow(2, 55) || value.value > pow(2, 55)-1){
            throw std::out_of_range("Int56 value exceeds 56 bit signed range");
        }

        const auto raw = static_cast<std::uint64_t>(value.value);

        return sdo::helpers::from_raw<std::uint64_t>(raw, 7);
    }

    template <> [[nodiscard]] inline std::vector<std::uint8_t> serialize<int64_t>(const int64_t& value)
    {
        return sdo::helpers::from_raw<std::uint64_t>(static_cast<std::uint64_t>(value), 8);
    }

    // -Extended unsigned Integer-

    template <> [[nodiscard]] inline std::vector<std::uint8_t> serialize<UInt24>(const UInt24& value)
    {
        if (value.value > pow(2, 24)-1){
            throw std::out_of_range("UInt24 value exceeds 24 bit unsigned range");
        }

        return sdo::helpers::from_raw<std::uint32_t>(value.value, 3);
    }

    template <> [[nodiscard]] inline std::vector<std::uint8_t> serialize<UInt40>(const UInt40& value)
    {
        if (value.value > pow(2, 40)-1){
            throw std::out_of_range("UInt40 value exceeds 40 bit unsigned range");
        }

        return sdo::helpers::from_raw<std::uint64_t>(value.value, 5);
    }

    template <> [[nodiscard]] inline std::vector<std::uint8_t> serialize<UInt48>(const UInt48& value)
    {
        if (value.value > pow(2, 48)-1){
            throw std::out_of_range("UInt48 value exceeds 48 bit unsigned range");
        }

        return sdo::helpers::from_raw<std::uint64_t>(value.value, 6);
    }

    template <> [[nodiscard]] inline std::vector<std::uint8_t> serialize<UInt56>(const UInt56& value)
    {
        if (value.value > pow(2, 56)-1){
            throw std::out_of_range("UInt56 value exceeds 56 bit unsigned range");
        }

        return sdo::helpers::from_raw<std::uint64_t>(value.value, 7);
    }

    template <> [[nodiscard]] inline std::vector<std::uint8_t> serialize<uint64_t>(const uint64_t& value)
    {
        return sdo::helpers::from_raw<std::uint64_t>(value, 8);
    }

    // -GUID-

    template <> [[nodiscard]] inline std::vector<std::uint8_t> serialize<Guid>(const Guid& value)
    {
        return std::vector<std::uint8_t>(value.bytes.begin(), value.bytes.end());
    }
}
