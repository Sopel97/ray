#pragma once

namespace ray
{
    struct AssumeNormalized {};

    namespace detail
    {
        template <typename T, typename OwnerT>
        struct OwnedReadOnlyProperty
        {
            friend OwnerT;

            constexpr operator const T&() const
            {
                return m_value;
            }

            void negate()
            {
                m_value = -m_value;
            }

        private:
            T m_value;

            constexpr OwnedReadOnlyProperty(const T& value) :
                m_value(value)
            {

            }

            constexpr OwnedReadOnlyProperty(T&& value) :
                m_value(std::move(value))
            {

            }

            constexpr OwnedReadOnlyProperty(const OwnedReadOnlyProperty&) = default;
            constexpr OwnedReadOnlyProperty(OwnedReadOnlyProperty&&) = default;

            constexpr OwnedReadOnlyProperty& operator=(const OwnedReadOnlyProperty&) = default;
            constexpr OwnedReadOnlyProperty& operator=(OwnedReadOnlyProperty&&) = default;

            constexpr T& operator=(const T& value)
            {
                return m_value = value;
            }

            constexpr T& operator=(T&& value)
            {
                return m_value = std::move(value);
            }
        };
    }
}
