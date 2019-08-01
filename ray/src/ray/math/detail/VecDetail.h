#pragma once

#include <utility>

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
    }
}
