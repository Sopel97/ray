#pragma once

#include <cstdint>
#include <iterator>

namespace ray
{
    template <typename IntT>
    struct IterableNumber
    {
        using difference_type = std::ptrdiff_t;
        using value_type = IntT;
        using const_reference = const IntT&;
        using const_pointer = const IntT*;
        using reference = IntT&;
        using pointer = IntT*;
        using iterator_category = std::random_access_iterator_tag;

        IterableNumber() noexcept : m_value{} {}
        IterableNumber(const IntT& value) noexcept : m_value(value) {}

        IterableNumber(const IterableNumber&) = default;
        IterableNumber(IterableNumber&&) = default;
        IterableNumber& operator=(const IterableNumber&) = default;
        IterableNumber& operator=(IterableNumber&&) = default;

        IterableNumber  operator++ (int) const { IterableNumber i = *this; operator++(); return i; }
        IterableNumber  operator-- (int) const { IterableNumber i = *this; operator--(); return i; }
        IterableNumber& operator++ () { ++m_value; return *this; }
        IterableNumber& operator-- () { --m_value; return *this; }

        IterableNumber  operator+  (int n) const { IterableNumber i = *this; i += n; return i; }
        IterableNumber  operator-  (int n) const { IterableNumber i = *this; i -= n; return i; }
        IterableNumber& operator+= (int n) { m_value += n; return *this; }
        IterableNumber& operator-= (int n) { m_value -= n; return *this; }

        difference_type operator-  (const IterableNumber& rhs) const { return m_value - rhs.value; }

        value_type      operator[] (int n) const { return m_value + n; }
        value_type      operator*  () const { return m_value; }

        bool            operator== (const IterableNumber& rhs) const noexcept { return m_value == rhs.m_value; }
        bool            operator<  (const IterableNumber& rhs) const noexcept { return m_value < rhs.m_value; }
        bool            operator>  (const IterableNumber& rhs) const noexcept { return m_value > rhs.m_value; }
        bool            operator<= (const IterableNumber& rhs) const noexcept { return m_value <= rhs.m_value; }
        bool            operator>= (const IterableNumber& rhs) const noexcept { return m_value >= rhs.m_value; }
        bool            operator!= (const IterableNumber& rhs) const noexcept { return m_value != rhs.m_value; }

    private:
        IntT m_value;
    };
}
