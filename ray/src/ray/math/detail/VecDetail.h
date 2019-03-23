#pragma once

#if defined(__clang__) || defined(__GNUC__) || defined(__GNUG__)

#define RAY_UNREACHABLE() (__builtin_unreachable())

#elif defined(_MSC_VER)

#define RAY_UNREACHABLE() (__assume(0))

#else

#define RAY_UNREACHABLE() ((void)0)

#endif

namespace ray
{
    struct AssumeNormalized {};

    namespace detail
    {
        template <typename T, typename OwnerT>
        struct OwnedReadOnlyProperty
        {
            friend OwnerT;

            [[nodiscard]] constexpr operator const T&() const
            {
                return m_value;
            }

            constexpr void negate()
            {
                m_value = -m_value;
            }

        private:
            T m_value;

            constexpr OwnedReadOnlyProperty(const T& value) noexcept :
                m_value(value)
            {

            }

            constexpr OwnedReadOnlyProperty(T&& value) noexcept :
                m_value(std::move(value))
            {

            }

            constexpr OwnedReadOnlyProperty(const OwnedReadOnlyProperty&) noexcept = default;
            constexpr OwnedReadOnlyProperty(OwnedReadOnlyProperty&&) noexcept = default;

            constexpr OwnedReadOnlyProperty& operator=(const OwnedReadOnlyProperty&) noexcept = default;
            constexpr OwnedReadOnlyProperty& operator=(OwnedReadOnlyProperty&&) noexcept = default;

            constexpr T& operator=(const T& value) noexcept
            {
                return m_value = value;
            }

            constexpr T& operator=(T&& value) noexcept
            {
                return m_value = std::move(value);
            }
        };

        template <typename VecT, typename ScalarT>
        struct ScalarExtractor
        {
            [[nodiscard]] static ScalarExtractor x()
            {
                return { 0 };
            }
            [[nodiscard]] static ScalarExtractor y()
            {
                return { 1 };
            }
            [[nodiscard]] static ScalarExtractor z()
            {
                return { 2 };
            }

            ScalarExtractor(int i) :
                m_index(i)
            {

            }

            [[nodiscard]] ScalarT extractFrom(const VecT& v) const
            {
                switch (m_index)
                {
                case 0:
                    return v.x;
                case 1:
                    return v.y;
                case 2:
                    return v.z;
                }

                RAY_UNREACHABLE();
            }

        private:
            int m_index;
        };
    }
}

#undef RAY_UNREACHABLE
