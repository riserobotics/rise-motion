namespace rise
{
    template <typename T, typename E> struct Result
    {
        T value{};
        E error{};
        bool hasValue = false;

        static Result ok(T v)
        {
            Result result;
            result.value = std::move(v);
            result.hasValue = true;
            return result;
        }

        static Result err(E e)
        {
            Result result;
            result.error = std::move(e);
            result.hasValue = false;
            return result;
        }

        explicit operator bool() const
        {
            return hasValue;
        }
    };
}