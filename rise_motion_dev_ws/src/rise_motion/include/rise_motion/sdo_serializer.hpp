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
#include "result.hpp"

namespace sdo
{
    enum class ErrorCode
    {
        None,
        InvalidSize,
        OutOfRange,
        UnsupportedType
    };

    struct Error
    {
        ErrorCode code = ErrorCode::None;
        std::string message = "";
    };


    // as equivalent of the EtherCAT TIME_OF_DAY data type
    struct TimeOfDay
    {
        std::uint32_t ms_since_midnight;
        std::uint16_t d_since_1984_01_01;

        TimeOfDay() = default;
    };

    // as equivalent of the EtherCAT TIME_DIFFERENCE data type
    struct TimeDifference
    {
        std::uint32_t ms;
        std::uint16_t d;

        TimeDifference() = default;
    };

    struct Int24
    {
        std::int32_t value;

        Int24() = default;

        Int24(std::int32_t v) : value(v)
        {}

        operator std::int32_t() const
        {
            return value;
        }
    };

    struct Int40
    {
        std::int64_t value;

        Int40() = default;

        Int40(std::int64_t v) : value(v)
        {}

        operator std::int64_t() const
        {
            return value;
        }
    };

    struct Int48
    {
        std::int64_t value;

        Int48() = default;

        Int48(std::int64_t v) : value(v)
        {}

        operator std::int64_t() const
        {
            return value;
        }
    };

    struct Int56
    {
        std::int64_t value;

        Int56() = default;

        Int56(std::int64_t v) : value(v)
        {}

        operator std::int64_t() const
        {
            return value;
        }
    };

    struct UInt24
    {
        std::uint32_t value;

        UInt24() = default;

        UInt24(std::uint32_t v) : value(v)
        {}

        operator std::uint32_t() const
        {
            return value;
        }
    };

    struct UInt40
    {
        std::uint64_t value;

        UInt40() = default;

        UInt40(std::uint64_t v) : value(v)
        {}

        operator std::uint64_t() const
        {
            return value;
        }
    };

    struct UInt48
    {
        std::uint64_t value;

        UInt48() = default;

        UInt48(std::uint64_t v) : value(v)
        {}

        operator std::uint64_t() const
        {
            return value;
        }
    };

    struct UInt56
    {
        std::uint64_t value;

        UInt56() = default;

        UInt56(std::uint64_t v) : value(v)
        {}

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
        {}

        operator std::string() const
        {
            return value;
        }
    };

    template <std::size_t N> struct WSTRING
    {
        std::u16string value;

        WSTRING() = default;

        WSTRING(std::u16string v) : value(std::move(v))
        {}

        operator std::u16string() const
        {
            return value;
        }
    };

    template <typename T, std::size_t N> struct ARRAY
    {
        std::vector<T> value;

        ARRAY() = default;

        ARRAY(std::vector<T> v) : value(std::move(v))
        {}

        operator std::vector<T>() const
        {
            return value;
        }
    };


    template <typename T> using DeserializeResult = rise::Result<T, sdo::Error>;
    using SerializeResult = rise::Result<std::vector<std::uint8_t>, sdo::Error>;

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

    using BITARR8   = std::uint8_t;
    using BITARR16  = std::uint16_t;
    using BITARR32  = std::uint32_t;

    using BIT1  = std::uint8_t;
    using BIT2  = std::uint8_t;
    using BIT3  = std::uint8_t;
    using BIT4  = std::uint8_t;
    using BIT5  = std::uint8_t;
    using BIT6  = std::uint8_t;
    using BIT7  = std::uint8_t;
    using BIT8  = std::uint8_t;
    using BIT9  = std::uint16_t;
    using BIT10  = std::uint16_t;
    using BIT11  = std::uint16_t;
    using BIT12  = std::uint16_t;
    using BIT13  = std::uint16_t;
    using BIT14  = std::uint16_t;
    using BIT15  = std::uint16_t;
    using BIT16  = std::uint16_t;

    using REAL32 = float;
    using REAL64 = double;

    using TIME_OF_DAY     = sdo::TimeOfDay;
    using TIME_DIFFERENCE = sdo::TimeDifference;

    using GUID   = sdo::Guid;
    using DOMAIN = std::vector<std::uint8_t>;

    template <std::size_t T> using VISIBLE_STRING = STRING<T>;
    template <std::size_t T> using UNICODE_STRING = WSTRING<T>;
    template <std::size_t N> using OKTET_STRING = ARRAY<std::uint8_t, N>;
    template <std::size_t N> using ARRAY_OF_USINT = ARRAY<std::uint8_t, N>;
    template <std::size_t N> using ARRAY_OF_UINT = ARRAY<std::uint16_t, N>;
    template <std::size_t N> using ARRAY_OF_INT = ARRAY<std::int16_t, N>;
    template <std::size_t N> using ARRAY_OF_SINT = ARRAY<std::int8_t, N>;
    template <std::size_t N> using ARRAY_OF_DINT = ARRAY<std::int32_t, N>;
    template <std::size_t N> using ARRAY_OF_UDINT = ARRAY<std::uint32_t, N>;
    template <std::size_t N> using ARRAY_OF_BITARR8 = ARRAY<std::uint8_t, N>;
    template <std::size_t N> using ARRAY_OF_BITARR16 = ARRAY<std::uint16_t, N>;
    template <std::size_t N> using ARRAY_OF_BITARR32 = ARRAY<std::uint32_t, N>;
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

    template <typename T> struct is_wstring_type : std::false_type {};
    template <std::size_t T> struct is_wstring_type<sdo::WSTRING<T>> : std::true_type {};

    template <typename T> struct wstring_size;
    template <std::size_t T>struct wstring_size<sdo::WSTRING<T>>
    {
        static constexpr std::size_t size = T;
    };

    template <typename T> struct is_array_type : std::false_type {};
    template <typename T, std::size_t N> 
        struct is_array_type<sdo::ARRAY<T, N>> : std::true_type {};
        
    template <typename T> struct array_size;
    template <typename T, std::size_t N>struct array_size<sdo::ARRAY<T, N>>
    {
        static constexpr std::size_t size = N;
    };

    template <typename T> struct elementType;
    template <typename T, std::size_t N>struct elementType<sdo::ARRAY<T, N>>
    {
        using type = T;
    };


    inline std::optional<sdo::Error> check_size(
        const std::vector<std::uint8_t>& blob, std::size_t expectedSize, const char* type)
    {
        if (blob.size() != expectedSize){
            return Error{
                ErrorCode::InvalidSize,
                std::string("deserialize<") + type + ">: expected " + std::to_string(expectedSize) +
                " bytes, got " + std::to_string(blob.size())};
        }

        return std::nullopt;
    }

    template <typename T> [[nodiscard]] inline DeserializeResult<T> to_raw(
        const std::vector<std::uint8_t>& blob, std::size_t numBytes, std::size_t offset = 0)
    {
        static_assert(std::is_unsigned_v<T>, "to_raw<T>: T must be unsigned");
        static_assert(sizeof(T) <= sizeof(std::uint64_t), "to_raw<T>: T can be max uint64_t");

        if (numBytes > sizeof(T))
        {
            return DeserializeResult<T>::err({
                ErrorCode::InvalidSize, "to_raw<T>: numBytes does not fit T"});
        }

        if (offset + numBytes > blob.size())
        {
            return DeserializeResult<T>::err({
                ErrorCode::InvalidSize, "to_raw<T>: blob has less bytes than numBytes"});
        }

        T raw = 0;

        for (std::size_t i = 0; i < numBytes; ++i)
        {
            raw |= static_cast<T>(blob[offset + i]) << (8 * i);
        }

        return DeserializeResult<T>::ok(raw);
    }

    template <typename T> [[nodiscard]] inline SerializeResult from_raw(
        const T& raw, std::size_t numBytes)
    {
        static_assert(std::is_unsigned_v<T>, "from_raw<T>: T must be unsigned");

        if (numBytes > sizeof(T))
        {
            SerializeResult::err({ErrorCode::InvalidSize, "from_raw<T>: numBytes does not fit T"});
        }

        std::vector<std::uint8_t> blob;
        blob.reserve(numBytes);

        for (std::size_t i = 0; i < numBytes; ++i)
        {
            blob.push_back(static_cast<std::uint8_t>((raw >> (8 * i)) & 0xFF));
        }

        return SerializeResult::ok(blob);
    }
}

namespace sdo
{
    // ---DESERIALIZATION---

    template <typename T> [[nodiscard]] inline DeserializeResult<T> deserialize(
        const std::vector<std::uint8_t>& blob)
    {
        using CleanT = std::remove_cv_t<std::remove_reference_t<T>>;

        if constexpr (sdo::helpers::is_array_type<CleanT>::value)
        {
            constexpr std::size_t numElements = sdo::helpers::array_size<CleanT>::size;
            using ElementT = typename sdo::helpers::elementType<CleanT>::type;
            constexpr std::size_t elementSize = sizeof(ElementT);
            constexpr std::size_t byteSize = numElements * elementSize;

            if (blob.size() > byteSize || blob.size() % elementSize != 0){
                return DeserializeResult<T>::err({
                    ErrorCode::InvalidSize, "deserialize<ARRAY<T, N>>: invalid blob size"});
            }

            T result{};
            result.value.reserve(blob.size() / elementSize);

            for (std::size_t i = 0; i < blob.size(); i += elementSize){
                if constexpr (elementSize == 1){
                    result.value.push_back(static_cast<ElementT>(blob[i]));
                }
                else if constexpr (elementSize == 2){
                    auto raw = sdo::helpers::to_raw<std::uint16_t>(blob, 2, i);
                    if (!raw){
                        return DeserializeResult<T>::err(raw.error);
                    }

                    result.value.push_back(static_cast<ElementT>(raw.value));
                }
                else if constexpr (elementSize == 4){
                    auto raw = sdo::helpers::to_raw<std::uint32_t>(blob, 4, i);
                    if (!raw){
                        return DeserializeResult<T>::err(raw.error);
                    }

                    if constexpr (std::is_same_v<ElementT, float>){
                        float element;
                        std::memcpy(&element, &raw.value, sizeof(element));
                        result.value.push_back(element);
                    }
                    else{
                        result.value.push_back(static_cast<ElementT>(raw.value));
                    }
                }
                else if constexpr (elementSize == 8){
                    auto raw = sdo::helpers::to_raw<std::uint64_t>(blob, 8, i);
                    if (!raw){
                        return DeserializeResult<T>::err(raw.error);
                    }

                    if constexpr (std::is_same_v<ElementT, double>){
                        double element;
                        std::memcpy(&element, &raw.value, sizeof(element));
                        result.value.push_back(element);
                    }
                    else{
                        result.value.push_back(static_cast<ElementT>(raw.value));
                    }
                }
                else{
                    return DeserializeResult<T>::err({
                        ErrorCode::UnsupportedType,
                        "deserialize<ARRAY<T, N>>: unsupported array type"});
                }
            }

            return DeserializeResult<T>::ok(std::move(result));
        }
        else if constexpr (sdo::helpers::is_wstring_type<CleanT>::value)
        {
            constexpr std::size_t size = sdo::helpers::wstring_size<CleanT>::size;

            if (blob.size() > size * 2 || blob.size() % 2 != 0){
                return DeserializeResult<T>::err({
                    ErrorCode::InvalidSize,
                    "deserialize<WSTRING<T>>: blob is larger than the expected size T"});
            }

            T str{};

            for (std::size_t i = 0; i < blob.size(); i += 2){
                std::uint16_t raw = 
                    static_cast<std::uint16_t>(blob[i]) |
                    static_cast<std::uint16_t>(blob[i + 1]) << 8;

                str.value.push_back(static_cast<char16_t>(raw));
            }

            // strip padding zeros
            while (!str.value.empty() && str.value.back() == u'\0'){
                str.value.pop_back();
            }

            return DeserializeResult<T>::ok(str);
        }
        else if constexpr (sdo::helpers::is_string_type<CleanT>::value)
        {
        
            constexpr std::size_t size = sdo::helpers::string_size<CleanT>::size;

            if (blob.size() > size){
                return DeserializeResult<T>::err({
                    ErrorCode::InvalidSize, 
                    "deserialize<STRING<T>>: blob is larger than the expected size T"});
            }

            T str{};

            str.value.assign(blob.begin(), blob.end());

            // strip padding zeros
            while (!str.value.empty() && str.value.back() == '\0'){
                str.value.pop_back();
            }

            return DeserializeResult<T>::ok(str);
        }
        else{
            static_assert(sdo::helpers::always_false<T>, "deserialize<T>: unsupported type");
        }
    }

    // -Boolean-

    template <> [[nodiscard]] inline DeserializeResult<bool> deserialize<bool>(
        const std::vector<std::uint8_t>& blob)
    {
        if (auto error = sdo::helpers::check_size(blob, 1, "bool")){
            return sdo::DeserializeResult<bool>::err(*error);
        }

        return DeserializeResult<bool>::ok(blob[0] != 0);
    }

    // -Signed Integer-

    template <> [[nodiscard]] inline DeserializeResult<std::int8_t> deserialize<std::int8_t>(
        const std::vector<std::uint8_t>& blob)
    {
        if (auto error = sdo::helpers::check_size(blob, 1, "int8_t")){
            return sdo::DeserializeResult<std::int8_t>::err(*error);
        }

        return DeserializeResult<std::int8_t>::ok(static_cast<std::int8_t>(blob[0]));
    }

    template <> [[nodiscard]] inline DeserializeResult<std::int16_t> deserialize<std::int16_t>(
        const std::vector<std::uint8_t>& blob)
    {
        if (auto error = sdo::helpers::check_size(blob, 2, "int16_t")){
            return sdo::DeserializeResult<std::int16_t>::err(*error);
        }

        auto raw = sdo::helpers::to_raw<std::uint16_t>(blob, 2);
        if(!raw){
            return DeserializeResult<std::int16_t>::err(raw.error);
        }

        auto result = static_cast<std::int16_t>(raw.value);

        return DeserializeResult<std::int16_t>::ok(result);
    }

    template <> [[nodiscard]] inline DeserializeResult<std::int32_t> deserialize<std::int32_t>(
        const std::vector<std::uint8_t>& blob)
    {
        if (auto error = sdo::helpers::check_size(blob, 4, "int32_t")){
            return sdo::DeserializeResult<std::int32_t>::err(*error);
        }

        auto raw = sdo::helpers::to_raw<std::uint32_t>(blob, 4);
        if(!raw){
            return DeserializeResult<std::int32_t>::err(raw.error);
        }

        auto result = static_cast<std::int32_t>(raw.value);

        return DeserializeResult<std::int32_t>::ok(result);
    }

    // -Unsigned Integer / raw data / bit arrays / bit strings-

    // use for UNSIGNED8, BYTE, BITARR8, BIT1-BIT8
    template <> [[nodiscard]] inline DeserializeResult<std::uint8_t> deserialize<std::uint8_t>(
        const std::vector<std::uint8_t>& blob)
    {
        if (auto error = sdo::helpers::check_size(blob, 1, "uint8_t")){
            return sdo::DeserializeResult<std::uint8_t>::err(*error);
        }

        return DeserializeResult<std::uint8_t>::ok(static_cast<std::uint8_t>(blob[0]));
    }

    // use for UNSIGNED16, WORD, BITARR16, BIT9-BIT16
    template <> [[nodiscard]] inline DeserializeResult<std::uint16_t> deserialize<std::uint16_t>(
        const std::vector<std::uint8_t>& blob)
    {
        if (auto error = sdo::helpers::check_size(blob, 2, "uint16_t")){
            return sdo::DeserializeResult<std::uint16_t>::err(*error);
        }

        return sdo::helpers::to_raw<std::uint16_t>(blob, 2);
    }

    // use for UNSIGNED32, DWORD, BITARR32
    template <> [[nodiscard]] inline DeserializeResult<std::uint32_t> deserialize<std::uint32_t>(
        const std::vector<std::uint8_t>& blob)
    {
        if (auto error = sdo::helpers::check_size(blob, 4, "uint32_t")){
            return sdo::DeserializeResult<std::uint32_t>::err(*error);
        }

        return sdo::helpers::to_raw<std::uint32_t>(blob, 4);
    }

    // -Floating Point-

    template <> [[nodiscard]] inline DeserializeResult<float> deserialize<float>(
        const std::vector<std::uint8_t>& blob)
    {
        if (auto error = sdo::helpers::check_size(blob, 4, "float")){
            return sdo::DeserializeResult<float>::err(*error);
        }

        auto raw = sdo::helpers::to_raw<std::uint32_t>(blob, 4);
        if(!raw){
            return DeserializeResult<float>::err(raw.error);
        }

        float value;
        std::memcpy(&value, &raw.value, sizeof(value));

        return DeserializeResult<float>::ok(value);
    }

    template <> [[nodiscard]] inline DeserializeResult<double> deserialize<double>(
        const std::vector<std::uint8_t>& blob)
    {
        if (auto error = sdo::helpers::check_size(blob, 8, "double")){
            return sdo::DeserializeResult<double>::err(*error);
        }

        auto raw = sdo::helpers::to_raw<std::uint64_t>(blob, 8);
        if(!raw){
            return DeserializeResult<double>::err(raw.error);
        }

        double value;
        std::memcpy(&value, &raw.value, sizeof(value));

        return DeserializeResult<double>::ok(value);
    }

    // -Time-

    template <> [[nodiscard]] inline DeserializeResult<TimeOfDay> deserialize<TimeOfDay>(
        const std::vector<std::uint8_t>& blob)
    {
        if (auto error = sdo::helpers::check_size(blob, 6, "TimeOfDay")){
            return sdo::DeserializeResult<TimeOfDay>::err(*error);
        }

        TimeOfDay value{};

        auto ms_raw = sdo::helpers::to_raw<std::uint32_t>(blob, 4);
        if(!ms_raw){
            return DeserializeResult<TimeOfDay>::err(ms_raw.error);
        }
        value.ms_since_midnight = ms_raw.value;

        auto d_raw = sdo::helpers::to_raw<std::uint16_t>(blob, 2, 4);
        if(!d_raw){
            return DeserializeResult<TimeOfDay>::err(d_raw.error);
        }
        value.d_since_1984_01_01 = d_raw.value;

        return DeserializeResult<TimeOfDay>::ok(value);
    }

    template <> [[nodiscard]] inline DeserializeResult<TimeDifference> deserialize<TimeDifference>(
        const std::vector<std::uint8_t>& blob)
    {
        if (auto error = sdo::helpers::check_size(blob, 6, "TimeDifference")){
            return sdo::DeserializeResult<TimeDifference>::err(*error);
        }

        TimeDifference value{};

        auto ms_raw =  sdo::helpers::to_raw<std::uint32_t>(blob, 4);
        if(!ms_raw){
            return DeserializeResult<TimeDifference>::err(ms_raw.error);
        }
        // the upper 4 bits of ms are reserved
        value.ms =  ms_raw.value & 0x0FFFFFFF;

        auto d_raw = sdo::helpers::to_raw<std::uint16_t>(blob, 2, 4);
        if(!d_raw){
            return DeserializeResult<TimeDifference>::err(d_raw.error);
        }
        value.d = d_raw.value;

        return DeserializeResult<TimeDifference>::ok(value);
    }

    // -Domain-
    // (returns raw blob as equivalent to the EtherCAT Domain data type)

    template <> [[nodiscard]] inline DeserializeResult<std::vector<std::uint8_t>> deserialize<std::vector<std::uint8_t>>(
        const std::vector<std::uint8_t>& blob)
    {
        return DeserializeResult<std::vector<std::uint8_t>>::ok(blob);
    }

    // -Extended Signed Integer-

    template <> [[nodiscard]] inline DeserializeResult<Int24> deserialize<Int24>(
        const std::vector<std::uint8_t>& blob)
    {
        if (auto error = sdo::helpers::check_size(blob, 3, "Int24")){
            return sdo::DeserializeResult<Int24>::err(*error);
        }

        auto raw = sdo::helpers::to_raw<std::uint32_t>(blob, 3);
        if(!raw){
            return DeserializeResult<Int24>::err(raw.error);
        }

        if (raw.value & 0x00800000){
            raw.value |= 0xFF000000;
        }

        return DeserializeResult<Int24>::ok(Int24{static_cast<std::int32_t>(raw.value)});
    }

    template <> [[nodiscard]] inline DeserializeResult<Int40> deserialize<Int40>(
        const std::vector<std::uint8_t>& blob)
    {
        if (auto error = sdo::helpers::check_size(blob, 5, "Int40")){
            return sdo::DeserializeResult<Int40>::err(*error);
        }

        auto raw = sdo::helpers::to_raw<std::uint64_t>(blob, 5);
        if(!raw){
            return DeserializeResult<Int40>::err(raw.error);
        }

        if (raw.value & 0x0000008000000000ULL)
        {
            raw.value |= 0xFFFFFF0000000000ULL;
        }

        return DeserializeResult<Int40>::ok(Int40{static_cast<std::int64_t>(raw.value)});
    }

    template <> [[nodiscard]] inline DeserializeResult<Int48> deserialize<Int48>(
        const std::vector<std::uint8_t>& blob)
    {
        if (auto error = sdo::helpers::check_size(blob, 6, "Int48")){
            return sdo::DeserializeResult<Int48>::err(*error);
        }

        auto raw = sdo::helpers::to_raw<std::uint64_t>(blob, 6);
        if(!raw){
            return DeserializeResult<Int48>::err(raw.error);
        }

        if (raw.value & 0x0000800000000000ULL)
        {
            raw.value |= 0xFFFF000000000000ULL;
        }

        return DeserializeResult<Int48>::ok(Int48{static_cast<std::int64_t>(raw.value)});
    }

    template <> [[nodiscard]] inline DeserializeResult<Int56> deserialize<Int56>(
        const std::vector<std::uint8_t>& blob)
    {
        if (auto error = sdo::helpers::check_size(blob, 7, "Int56")){
            return sdo::DeserializeResult<Int56>::err(*error);
        }

        auto raw = sdo::helpers::to_raw<std::uint64_t>(blob, 7);
        if(!raw){
            return DeserializeResult<Int56>::err(raw.error);
        }

        if (raw.value & 0x0080000000000000ULL)
        {
            raw.value |= 0xFF00000000000000ULL;
        }

        return DeserializeResult<Int56>::ok(Int56{static_cast<std::int64_t>(raw.value)});
    }

    template <> [[nodiscard]] inline DeserializeResult<std::int64_t> deserialize<std::int64_t>(
        const std::vector<std::uint8_t>& blob)
    {
        if (auto error = sdo::helpers::check_size(blob, 8, "int64_t")){
            return sdo::DeserializeResult<std::int64_t>::err(*error);
        }

        auto raw = sdo::helpers::to_raw<std::uint64_t>(blob, 8);
        if(!raw){
            return DeserializeResult<std::int64_t>::err(raw.error);
        }

        return DeserializeResult<std::int64_t>::ok(static_cast<std::int64_t>(raw.value));
    }

    // -Extended Unsigned Integer-

    template <> [[nodiscard]] inline DeserializeResult<UInt24> deserialize<UInt24>(
        const std::vector<std::uint8_t>& blob)
    {
        if (auto error = sdo::helpers::check_size(blob, 3, "UInt24")){
            return sdo::DeserializeResult<UInt24>::err(*error);
        }

        auto raw = sdo::helpers::to_raw<std::uint32_t>(blob, 3);
        if(!raw){
            return DeserializeResult<UInt24>::err(raw.error);
        }

        return DeserializeResult<UInt24>::ok(UInt24{raw.value});
    }

    template <> [[nodiscard]] inline DeserializeResult<UInt40> deserialize<UInt40>(
        const std::vector<std::uint8_t>& blob)
    {
        if (auto error = sdo::helpers::check_size(blob, 5, "UInt40")){
            return sdo::DeserializeResult<UInt40>::err(*error);
        }

        auto raw = sdo::helpers::to_raw<std::uint64_t>(blob, 5);
        if(!raw){
            return DeserializeResult<UInt40>::err(raw.error);
        }

        return DeserializeResult<UInt40>::ok(UInt40{raw.value});
    }

    template <> [[nodiscard]] inline DeserializeResult<UInt48> deserialize<UInt48>(
        const std::vector<std::uint8_t>& blob)
    {
        if (auto error = sdo::helpers::check_size(blob, 6, "UInt48")){
            return sdo::DeserializeResult<UInt48>::err(*error);
        }

        auto raw = sdo::helpers::to_raw<std::uint64_t>(blob, 6);
        if(!raw){
            return DeserializeResult<UInt48>::err(raw.error);
        }

        return DeserializeResult<UInt48>::ok(UInt48{raw.value});
    }

    template <> [[nodiscard]] inline DeserializeResult<UInt56> deserialize<UInt56>(
        const std::vector<std::uint8_t>& blob)
    {
        if (auto error = sdo::helpers::check_size(blob, 7, "UInt56")){
            return sdo::DeserializeResult<UInt56>::err(*error);
        }

        auto raw = sdo::helpers::to_raw<std::uint64_t>(blob, 7);
        if(!raw){
            return DeserializeResult<UInt56>::err(raw.error);
        }

        return DeserializeResult<UInt56>::ok(UInt56{raw.value});
    }

    template <> [[nodiscard]] inline DeserializeResult<std::uint64_t> deserialize<std::uint64_t>(
        const std::vector<std::uint8_t>& blob)
    {
        if (auto error = sdo::helpers::check_size(blob, 8, "uint64_t")){
            return sdo::DeserializeResult<std::uint64_t>::err(*error);
        }

        return sdo::helpers::to_raw<std::uint64_t>(blob, 8);
    }

    // -GUID-

    template <> [[nodiscard]] inline DeserializeResult<Guid> deserialize<Guid>(
        const std::vector<std::uint8_t>& blob)
    {
        if (auto error = sdo::helpers::check_size(blob, 16, "Guid")){
            return sdo::DeserializeResult<Guid>::err(*error);
        }

        Guid value{};

        for (std::size_t i = 0; i < value.bytes.size(); ++i)
        {
            value.bytes[i] = blob[i];
        }

        return DeserializeResult<Guid>::ok(value);
    }


    // ---SERIALIZATION---

    template <typename T> SerializeResult serialize(const T& value)
    {
        using CleanT = std::remove_cv_t<std::remove_reference_t<T>>;

        if constexpr (sdo::helpers::is_array_type<CleanT>::value)
        {
            constexpr std::size_t numElements = sdo::helpers::array_size<CleanT>::size;
            using ElementT = typename sdo::helpers::elementType<CleanT>::type;
            constexpr std::size_t elementSize = sizeof(ElementT);
            constexpr std::size_t byteSize = numElements * elementSize;

            if (value.value.size() > numElements){
                return SerializeResult::err({
                    ErrorCode::OutOfRange,
                    "serialize<ARRAY<T, N>>: array is longer than size N"
                });
            }

            std::vector<std::uint8_t> blob;
            blob.reserve(byteSize);

            for (const auto& element : value.value){
                if constexpr (elementSize == 1){
                    const auto raw = static_cast<std::uint8_t>(element);

                    blob.push_back(raw);
                }
                else if constexpr (elementSize == 2){
                    const auto raw = static_cast<std::uint16_t>(element);

                    blob.push_back(static_cast<std::uint8_t>(raw & 0xFF));
                    blob.push_back(static_cast<std::uint8_t>((raw >> 8) & 0xFF));
                }
                else if constexpr (elementSize == 4){
                    std::uint32_t raw;

                    if constexpr (std::is_same_v<ElementT, float>){
                        std::memcpy(&raw, &element, sizeof(raw));
                    }
                    else{
                        raw = static_cast<std::uint32_t>(element);
                    }

                    blob.push_back(static_cast<std::uint8_t>(raw & 0xFF));
                    blob.push_back(static_cast<std::uint8_t>((raw >> 8) & 0xFF));
                    blob.push_back(static_cast<std::uint8_t>((raw >> 16) & 0xFF));
                    blob.push_back(static_cast<std::uint8_t>((raw >> 24) & 0xFF));
                }
                else if constexpr (elementSize == 8){
                    std::uint64_t raw;

                    if constexpr (std::is_same_v<ElementT, double>){
                        std::memcpy(&raw, &element, sizeof(raw));
                    }
                    else{
                        raw = static_cast<std::uint64_t>(element);
                    }

                    blob.push_back(static_cast<std::uint8_t>(raw & 0xFF));
                    blob.push_back(static_cast<std::uint8_t>((raw >> 8) & 0xFF));
                    blob.push_back(static_cast<std::uint8_t>((raw >> 16) & 0xFF));
                    blob.push_back(static_cast<std::uint8_t>((raw >> 24) & 0xFF));
                    blob.push_back(static_cast<std::uint8_t>((raw >> 32) & 0xFF));
                    blob.push_back(static_cast<std::uint8_t>((raw >> 40) & 0xFF));
                    blob.push_back(static_cast<std::uint8_t>((raw >> 48) & 0xFF));
                    blob.push_back(static_cast<std::uint8_t>((raw >> 56) & 0xFF));
                }
                else{
                    return SerializeResult::err({
                        ErrorCode::UnsupportedType,
                        "serialize<ARRAY<T, N>>: unsupported array type"});
                }
            }

            blob.resize(byteSize, 0);

            return SerializeResult::ok(std::move(blob));
        }
        else if constexpr (sdo::helpers::is_wstring_type<CleanT>::value)
        {
            constexpr std::size_t size = sdo::helpers::wstring_size<CleanT>::size;

            if (value.value.size() > size)
            {
                return SerializeResult::err({
                    ErrorCode::InvalidSize, "serialize<WSTRING<T>>: string is longer than size T"});
            }

            std::vector<std::uint8_t> blob;
            blob.reserve(size * 2);

            for (char16_t c : value.value)
            {
                std::uint16_t raw = static_cast<std::uint16_t>(c);

                blob.push_back(static_cast<std::uint8_t>(raw & 0xFF));
                blob.push_back(static_cast<std::uint8_t>((raw >> 8) & 0xFF));
            }

            blob.resize(size * 2, '\0');

            return SerializeResult::ok(std::move(blob));
        }
        else if constexpr (sdo::helpers::is_string_type<CleanT>::value)
        {

            constexpr std::size_t size = sdo::helpers::string_size<T>::size;

            if (value.value.size() > size){
                return SerializeResult::err({
                    ErrorCode::InvalidSize, "serialize<STRING<T>>: string is longer than size T"});
            }

            std::vector<std::uint8_t> blob(value.value.begin(), value.value.end());
            blob.resize(size, '\0');

            return SerializeResult::ok(blob);
        }
        else{
            static_assert(sdo::helpers::always_false<T>, "serialize<T>: unsupported type");
        }
        return{};
    }

    // -Boolean-

    template <> [[nodiscard]] inline SerializeResult serialize<bool>(const bool& value)
    {
        return SerializeResult::ok({static_cast<std::uint8_t>(value ? 0xFF : 0)});
    }

    // -Signed Integer-

    template <> [[nodiscard]] inline SerializeResult serialize<std::int8_t>(const std::int8_t& value)
    {
        return SerializeResult::ok({static_cast<std::uint8_t>(value)});
    }

    template <> [[nodiscard]] inline SerializeResult serialize<std::int16_t>(const std::int16_t& value)
    {
        return sdo::helpers::from_raw<std::uint16_t>(static_cast<std::uint16_t>(value), 2);
    }

    template <> [[nodiscard]] inline SerializeResult serialize<std::int32_t>(const std::int32_t& value)
    {
        return sdo::helpers::from_raw<std::uint32_t>(static_cast<std::uint32_t>(value), 4);
    }

    // -Unsigned Integer / raw data / bit arrays / bit strings-

    // use for UNSIGNED8, BYTE, BITARR8, BIT1-BIT8
    template <> [[nodiscard]] inline SerializeResult serialize<std::uint8_t>(const std::uint8_t& value)
    {
        return SerializeResult::ok({value});
    }

    // use for UNSIGNED16, WORD, BITARR16, BIT9-BIT16
    template <> [[nodiscard]] inline SerializeResult serialize<std::uint16_t>(const std::uint16_t& value)
    {
        return sdo::helpers::from_raw<std::uint16_t>(value, 2);
    }

    // use for UNSIGNED32, DWORD, BITARR32

    template <> [[nodiscard]] inline SerializeResult serialize<std::uint32_t>(const std::uint32_t& value)
    {
        return sdo::helpers::from_raw<std::uint32_t>(value, 4);
    }

    // -Floating Point-

    template <> [[nodiscard]] inline SerializeResult serialize<float>(const float& value)
    {
        std::uint32_t raw;
        std::memcpy(&raw, &value, sizeof(raw));

        return sdo::helpers::from_raw<std::uint32_t>(raw, 4);
    }

    template <> [[nodiscard]] inline SerializeResult serialize<double>(const double& value)
    {
        std::uint64_t raw;
        std::memcpy(&raw, &value, sizeof(raw));

        return sdo::helpers::from_raw<std::uint64_t>(raw, 8);
    }

    // -Time-

    template <> [[nodiscard]] inline SerializeResult serialize<TimeOfDay>(const TimeOfDay& value)
    {
        return SerializeResult::ok({
            static_cast<std::uint8_t>(value.ms_since_midnight & 0xFF),
            static_cast<std::uint8_t>((value.ms_since_midnight >> 8) & 0xFF),
            static_cast<std::uint8_t>((value.ms_since_midnight >> 16) & 0xFF),
            static_cast<std::uint8_t>((value.ms_since_midnight >> 24) & 0xFF),

            static_cast<std::uint8_t>(value.d_since_1984_01_01 & 0xFF),
            static_cast<std::uint8_t>((value.d_since_1984_01_01 >> 8) & 0xFF)
        });
    }

    template <> [[nodiscard]] inline SerializeResult serialize<TimeDifference>(const TimeDifference& value)
    {
        if (value.ms > 0x0FFFFFFF)
        {
            SerializeResult::err({
                ErrorCode::OutOfRange, 
                "serialize<TimeDifference>: TimeDifference.ms exceeds 28 bit range"});
        }

        return SerializeResult::ok({
            static_cast<std::uint8_t>(value.ms & 0xFF),
            static_cast<std::uint8_t>((value.ms >> 8) & 0xFF),
            static_cast<std::uint8_t>((value.ms >> 16) & 0xFF),
            static_cast<std::uint8_t>((value.ms >> 24) & 0xFF),

            static_cast<std::uint8_t>(value.d & 0xFF),
            static_cast<std::uint8_t>((value.d >> 8) & 0xFF)
        });
    }

    // -Domain-
    // (returns raw blob as equivalent to the EtherCAT Domain data type)

    template <> [[nodiscard]] inline SerializeResult serialize<std::vector<std::uint8_t>>(
        const std::vector<std::uint8_t>& value)
    {
        return SerializeResult::ok(value);
    }

    // -Extended Signed Integer-

    template <> [[nodiscard]] inline SerializeResult serialize<Int24>(const Int24& value)
    {
        if (value.value < -pow(2, 23) || value.value > pow(2, 23)-1){
            SerializeResult::err({ErrorCode::OutOfRange, "Int24 value exceeds 24 bit signed range"});
        }

        const auto raw = static_cast<std::uint32_t>(value.value);

        return sdo::helpers::from_raw<std::uint32_t>(raw, 3);
    }

    template <> [[nodiscard]] inline SerializeResult serialize<Int40>(const Int40& value)
    {
        if (value.value < -pow(2, 39) || value.value > pow(2, 39)-1){
            SerializeResult::err({ErrorCode::OutOfRange, "Int40 value exceeds 40 bit signed range"});
        }

        const auto raw = static_cast<std::uint64_t>(value.value);

        return sdo::helpers::from_raw<std::uint64_t>(raw, 5);
    }

    template <> [[nodiscard]] inline SerializeResult serialize<Int48>(const Int48& value)
    {
        if (value.value < -pow(2, 47) || value.value > pow(2, 47)-1){
            SerializeResult::err({ErrorCode::OutOfRange, "Int48 value exceeds 48 bit signed range"});
        }

        const auto raw = static_cast<std::uint64_t>(value.value);

        return sdo::helpers::from_raw<std::uint64_t>(raw, 6);
    }

    template <> [[nodiscard]] inline SerializeResult serialize<Int56>(const Int56& value)
    {
        if (value.value < -pow(2, 55) || value.value > pow(2, 55)-1){
            SerializeResult::err({ErrorCode::OutOfRange, "Int56 value exceeds 56 bit signed range"});
        }

        const auto raw = static_cast<std::uint64_t>(value.value);

        return sdo::helpers::from_raw<std::uint64_t>(raw, 7);
    }

    template <> [[nodiscard]] inline SerializeResult serialize<int64_t>(const int64_t& value)
    {
        return sdo::helpers::from_raw<std::uint64_t>(static_cast<std::uint64_t>(value), 8);
    }

    // -Extended unsigned Integer-

    template <> [[nodiscard]] inline SerializeResult serialize<UInt24>(const UInt24& value)
    {
        if (value.value > pow(2, 24)-1){
            SerializeResult::err({
                ErrorCode::OutOfRange, "UInt24 value exceeds 24 bit unsigned range"});
        }

        return sdo::helpers::from_raw<std::uint32_t>(value.value, 3);
    }

    template <> [[nodiscard]] inline SerializeResult serialize<UInt40>(const UInt40& value)
    {
        if (value.value > pow(2, 40)-1){
            SerializeResult::err({
                ErrorCode::OutOfRange, "UInt40 value exceeds 40 bit unsigned range"});
        }

        return sdo::helpers::from_raw<std::uint64_t>(value.value, 5);
    }

    template <> [[nodiscard]] inline SerializeResult serialize<UInt48>(const UInt48& value)
    {
        if (value.value > pow(2, 48)-1){
            SerializeResult::err({
                ErrorCode::OutOfRange, "UInt48 value exceeds 48 bit unsigned range"});
        }

        return sdo::helpers::from_raw<std::uint64_t>(value.value, 6);
    }

    template <> [[nodiscard]] inline SerializeResult serialize<UInt56>(const UInt56& value)
    {
        if (value.value > pow(2, 56)-1){
            SerializeResult::err({
                ErrorCode::OutOfRange, "UInt56 value exceeds 56 bit unsigned range"});
        }

        return sdo::helpers::from_raw<std::uint64_t>(value.value, 7);
    }

    template <> [[nodiscard]] inline SerializeResult serialize<uint64_t>(const uint64_t& value)
    {
        return sdo::helpers::from_raw<std::uint64_t>(value, 8);
    }

    // -GUID-

    template <> [[nodiscard]] inline SerializeResult serialize<Guid>(const Guid& value)
    {
        return SerializeResult::ok(std::vector<std::uint8_t>(value.bytes.begin(), value.bytes.end()));
    }
}
