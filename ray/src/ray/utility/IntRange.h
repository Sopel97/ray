#pragma once

#include <cstdint>
#include <iterator>
#include <type_traits>

namespace ray
{
    template <typename IntT>
    struct IntIterator
    {
        static_assert(std::is_integral_v<IntT> && std::is_signed_v<IntT>, "Must be based on a signed integer");
        using IntType = IntT;

        using difference_type = IntType;
        using value_type = IntType;
        using const_reference = const value_type&;
        using const_pointer = const value_type*;
        using reference = value_type & ;
        using pointer = value_type * ;
        using iterator_category = std::random_access_iterator_tag;

        constexpr IntIterator() noexcept :
            m_value{}
        {
        }

        constexpr IntIterator(const IntT& value) noexcept :
            m_value(value)
        {
        }

        constexpr IntIterator(const IntIterator&) noexcept = default;
        constexpr IntIterator(IntIterator&&) noexcept = default;
        constexpr IntIterator& operator=(const IntIterator&) noexcept = default;
        constexpr IntIterator& operator=(IntIterator&&) noexcept = default;

        constexpr IntIterator operator++(int) const
        {
            IntIterator i = *this;
            operator++();
            return i;
        }
        constexpr IntIterator operator--(int) const
        {
            IntIterator i = *this;
            operator--();
            return i;
        }

        constexpr IntIterator& operator++()
        {
            ++m_value;
            return *this;
        }
        constexpr IntIterator& operator--()
        {
            --m_value;
            return *this;
        }

        [[nodiscard]] constexpr IntIterator operator+(difference_type n) const
        {
            IntIterator i = *this;
            i += n;
            return i;
        }
        [[nodiscard]] constexpr IntIterator operator-(difference_type n) const
        {
            IntIterator i = *this;
            i -= n;
            return i;
        }

        constexpr IntIterator& operator+=(difference_type n)
        {
            m_value += n;
            return *this;
        }
        constexpr IntIterator& operator-=(difference_type n)
        {
            m_value -= n;
            return *this;
        }

        [[nodiscard]] constexpr difference_type operator-(const IntIterator& rhs) const
        {
            return m_value - rhs.m_value;
        }

        [[nodiscard]] constexpr value_type operator[](difference_type n) const
        {
            return m_value + n;
        }
        [[nodiscard]] constexpr value_type operator*() const
        {
            return m_value;
        }

        [[nodiscard]] constexpr bool operator==(const IntIterator& rhs) const noexcept
        {
            return m_value == rhs.m_value;
        }

        [[nodiscard]] constexpr bool operator<(const IntIterator& rhs) const noexcept
        {
            return m_value < rhs.m_value;
        }
        [[nodiscard]] constexpr bool operator>(const IntIterator& rhs) const noexcept
        {
            return m_value > rhs.m_value;
        }
        [[nodiscard]] constexpr bool operator<=(const IntIterator& rhs) const noexcept
        {
            return m_value <= rhs.m_value;
        }
        [[nodiscard]] constexpr bool operator>=(const IntIterator& rhs) const noexcept
        {
            return m_value >= rhs.m_value;
        }
        [[nodiscard]] constexpr bool operator!=(const IntIterator& rhs) const noexcept
        {
            return m_value != rhs.m_value;
        }

    private:
        IntType m_value;
    };

    template <typename IntT>
    struct IntIteratorWithStep
    {
        static_assert(std::is_integral_v<IntT> && std::is_signed_v<IntT>, "Must be based on a signed integer");
        using IntType = IntT;

        using difference_type = IntType;
        using value_type = IntType;
        using const_reference = const value_type&;
        using const_pointer = const value_type*;
        using reference = value_type & ;
        using pointer = value_type * ;
        using iterator_category = std::random_access_iterator_tag;

        constexpr IntIteratorWithStep() noexcept :
            m_value{},
            m_step(1)
        {
        }
        constexpr IntIteratorWithStep(IntType value, IntType step) noexcept :
            m_value(value),
            m_step(step)
        {
        }

        constexpr IntIteratorWithStep(const IntIteratorWithStep&) noexcept = default;
        constexpr IntIteratorWithStep(IntIteratorWithStep&&) noexcept = default;
        constexpr IntIteratorWithStep& operator=(const IntIteratorWithStep&) noexcept = default;
        constexpr IntIteratorWithStep& operator=(IntIteratorWithStep&&) noexcept = default;

        constexpr IntIteratorWithStep operator++(int) const
        {
            IntIteratorWithStep i = *this;
            operator++();
            return i;
        }
        constexpr IntIteratorWithStep operator--(int) const
        {
            IntIteratorWithStep i = *this;
            operator--();
            return i;
        }
        constexpr IntIteratorWithStep& operator++()
        {
            m_value += m_step;
            return *this;
        }
        constexpr IntIteratorWithStep& operator--()
        {
            m_value -= m_step;
            return *this;
        }

        [[nodiscard]] constexpr IntIteratorWithStep operator+(difference_type n) const
        {
            IntIteratorWithStep i = *this;
            i += n;
            return i;
        }
        [[nodiscard]] constexpr IntIteratorWithStep operator-(difference_type n) const
        {
            IntIteratorWithStep i = *this;
            i -= n;
            return i;
        }
        constexpr IntIteratorWithStep& operator+=(difference_type n)
        {
            m_value += n * m_step;
            return *this;
        }
        constexpr IntIteratorWithStep& operator-=(difference_type n)
        {
            m_value -= n * m_step;
            return *this;
        }

        [[nodiscard]] constexpr difference_type operator-(const IntIteratorWithStep& rhs) const
        {
            return (m_value - rhs.m_value) / m_step;
        }

        [[nodiscard]] constexpr value_type operator[](difference_type n) const
        {
            return m_value + n * m_step;
        }
        [[nodiscard]] constexpr value_type operator*() const
        {
            return m_value;
        }

        [[nodiscard]] constexpr bool operator==(const IntIteratorWithStep& rhs) const noexcept
        {
            return m_value == rhs.m_value;
        }
        [[nodiscard]] constexpr bool operator<(const IntIteratorWithStep& rhs) const noexcept
        {
            return m_value < rhs.m_value;
        }
        [[nodiscard]] constexpr bool operator>(const IntIteratorWithStep& rhs) const noexcept
        {
            return m_value > rhs.m_value;
        }
        [[nodiscard]] constexpr bool operator<=(const IntIteratorWithStep& rhs) const noexcept
        {
            return m_value <= rhs.m_value;
        }
        [[nodiscard]] constexpr bool operator>=(const IntIteratorWithStep& rhs) const noexcept
        {
            return m_value >= rhs.m_value;
        }
        [[nodiscard]] constexpr bool operator!=(const IntIteratorWithStep& rhs) const noexcept
        {
            return m_value != rhs.m_value;
        }

    private:
        IntType m_value;
        IntType m_step;
    };

    template <typename IntT, bool HasStepV = false>
    struct IntRange
    {
        static_assert(std::is_integral_v<IntT> && std::is_signed_v<IntT>, "Must be based on a signed integer");
        using IntType = IntT;

        using iterator = IntIterator<IntT>;
        using const_iterator = IntIterator<IntT>;

        constexpr IntRange(IntType end) noexcept :
            m_begin(0),
            m_end(end)
        {
        }

        constexpr IntRange(IntType begin, IntType end) noexcept :
            m_begin(begin),
            m_end(end)
        {
        }

        [[nodiscard]] constexpr iterator begin()
        {
            return const_iterator(m_begin);
        }

        [[nodiscard]] constexpr const_iterator begin() const
        {
            return const_iterator(m_begin);
        }

        [[nodiscard]] constexpr const_iterator cbegin() const
        {
            return const_iterator(m_begin);
        }

        [[nodiscard]] constexpr iterator end()
        {
            return const_iterator(m_end);
        }

        [[nodiscard]] constexpr const_iterator end() const
        {
            return const_iterator(m_end);
        }

        [[nodiscard]] constexpr const_iterator cend() const
        {
            return const_iterator(m_end);
        }

    private:
        IntType m_begin;
        IntType m_end;

    };

    template <typename IntT>
    struct IntRange<IntT, true>
    {
        static_assert(std::is_integral_v<IntT> && std::is_signed_v<IntT>, "Must be based on a signed integer");
        using IntType = IntT;

        using iterator = IntIteratorWithStep<IntT>;
        using const_iterator = IntIteratorWithStep<IntT>;

        constexpr IntRange(IntType begin, IntType end, IntType step) noexcept :
            m_begin(begin),
            m_end(adjustedEnd(begin, end, step)),
            m_step(step)
        {
        }

        [[nodiscard]] constexpr iterator begin()
        {
            return const_iterator(m_begin, m_step);
        }

        [[nodiscard]] constexpr const_iterator begin() const
        {
            return const_iterator(m_begin, m_step);
        }

        [[nodiscard]] constexpr const_iterator cbegin() const
        {
            return const_iterator(m_begin, m_step);
        }

        [[nodiscard]] constexpr iterator end()
        {
            return const_iterator(m_end, m_step);
        }

        [[nodiscard]] constexpr const_iterator end() const
        {
            return const_iterator(m_end, m_step);
        }

        [[nodiscard]] constexpr const_iterator cend() const
        {
            return const_iterator(m_end, m_step);
        }

    private:
        IntType m_begin;
        IntType m_end;
        IntType m_step;

        [[nodiscard]] constexpr IntType adjustedEnd(IntType begin, IntType end, IntType step)
        {
            return begin + ((end - begin + step - 1) / step * step);
        }
    };

    template<typename IntT, bool HasStepV = false> IntRange(const IntT&)->IntRange<IntT, false>;
    template<typename IntT, bool HasStepV = false> IntRange(const IntT&, const IntT&)->IntRange<IntT, false>;
    template<typename IntT, bool HasStepV = false> IntRange(const IntT&, const IntT&, const IntT&)->IntRange<IntT, true>;

    static_assert(std::is_same_v<decltype(IntRange(1)), IntRange<int, false>>);
    static_assert(std::is_same_v<decltype(IntRange(0, 1)), IntRange<int, false>>);
    static_assert(std::is_same_v<decltype(IntRange(0, 1, 2)), IntRange<int, true>>);
}
