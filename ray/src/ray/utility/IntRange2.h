#pragma once

#include <cstdint>
#include <iterator>
#include <type_traits>

#include <ray/math/Vec2.h>

namespace ray
{
    template <typename IntT>
    struct IntIterator2
    {
        static_assert(std::is_integral_v<IntT> && std::is_signed_v<IntT>, "Must be based on a signed integer");
        using IntType = IntT;
        using PointType = Point2<IntType>;

        using difference_type = IntType;
        using value_type = PointType;
        using const_reference = const value_type&;
        using const_pointer = const value_type*;
        using reference = value_type & ;
        using pointer = value_type * ;
        using iterator_category = std::random_access_iterator_tag;

        IntIterator2() noexcept = default;
        IntIterator2(const PointType& begin, IntType minX, IntType maxX) noexcept :
            m_coords(begin),
            m_minX(minX),
            m_maxX(maxX)
        {
        }

        IntIterator2(const IntIterator2&) noexcept = default;
        IntIterator2(IntIterator2&&) noexcept = default;
        IntIterator2& operator=(const IntIterator2&) noexcept = default;
        IntIterator2& operator=(IntIterator2&&) noexcept = default;

        IntIterator2 operator++(int) const
        {
            IntIterator2 i = *this;
            operator++();
            return i;
        }
        IntIterator2 operator--(int) const
        {
            IntIterator2 i = *this;
            operator--();
            return i;
        }

        IntIterator2& operator++()
        {
            ++m_coords.x;
            if (m_coords.x > m_maxX)
            {
                ++m_coords.y;
                m_coords.x = m_minX;
            };
            return *this;
        }
        IntIterator2& operator--()
        {
            --m_coords.x;
            if (m_coords.x < m_minX)
            {
                --m_coords.y;
                m_coords.x = m_maxX;
            };
            return *this;
        }

        [[nodiscard]] IntIterator2 operator+(difference_type n) const
        {
            IntIterator2 i = *this;
            i += n;
            return i;
        }
        [[nodiscard]] IntIterator2 operator-(difference_type n) const
        {
            IntIterator2 i = *this;
            i -= n;
            return i;
        }

        IntIterator2& operator+=(difference_type n)
        {
            const IntType width = m_maxX - m_minX + 1;
            m_coords.y += n / width;
            m_coords.x += n % width;
            if (m_coords.x > m_maxX)
            {
                ++m_coords.y;
                m_coords.x -= width;
            }
            return *this;
        }
        IntIterator2& operator-=(difference_type n)
        {
            const IntType width = m_maxX - m_minX + 1;
            m_coords.y -= n / width;
            m_coords.x -= n % width;
            if (m_coords.x < m_minX)
            {
                --m_coords.y;
                m_coords.x += width;
            }
            return *this;
        }

        [[nodiscard]] difference_type operator-(const IntIterator2& rhs) const
        {
            const IntType width = m_maxX - m_minX + 1;
            return (m_coords.y * width + m_coords.x) - (rhs.m_coords.y * width + rhs.m_coords.x);
        }

        [[nodiscard]] value_type operator[](difference_type n) const
        {
            return m_coords + n;
        }
        [[nodiscard]] value_type operator*() const
        {
            return m_coords;
        }

        [[nodiscard]] bool operator==(const IntIterator2& rhs) const noexcept
        {
            return m_coords.x == rhs.m_coords.x && m_coords.y == rhs.m_coords.y;
        }
        [[nodiscard]] bool operator<(const IntIterator2& rhs) const noexcept
        {
            return *this - rhs < 0;
        }
        [[nodiscard]] bool operator>(const IntIterator2& rhs) const noexcept
        {
            return *this - rhs > 0;
        }
        [[nodiscard]] bool operator<=(const IntIterator2& rhs) const noexcept
        {
            return *this - rhs <= 0;
        }
        [[nodiscard]] bool operator>=(const IntIterator2& rhs) const noexcept
        {
            return *this - rhs >= 0;
        }
        [[nodiscard]] bool operator!=(const IntIterator2& rhs) const noexcept
        {
            return !operator==(rhs);
        }

    private:
        PointType m_coords;
        IntType m_minX;
        IntType m_maxX;
    };

    // we can assume that after going past the maxX it resets to minX, because the width is properly aligned
    template <typename IntT>
    struct IntIterator2WithStep
    {
        static_assert(std::is_integral_v<IntT> && std::is_signed_v<IntT>, "Must be based on a signed integer");
        using IntType = IntT;
        using PointType = Point2<IntType>;
        using VecType = Vec2<IntType>;

        using difference_type = IntType;
        using value_type = PointType;
        using const_reference = const value_type&;
        using const_pointer = const value_type*;
        using reference = value_type & ;
        using pointer = value_type * ;
        using iterator_category = std::random_access_iterator_tag;

        constexpr IntIterator2WithStep() noexcept = default;
        constexpr IntIterator2WithStep(const PointType& begin, IntType minX, IntType maxX, const VecType& step) noexcept :
            m_coords(begin),
            m_minX(minX),
            m_maxX(maxX),
            m_step(step)
        {
        }

        constexpr IntIterator2WithStep(const IntIterator2WithStep&) noexcept = default;
        constexpr IntIterator2WithStep(IntIterator2WithStep&&) noexcept = default;
        constexpr IntIterator2WithStep& operator=(const IntIterator2WithStep&) noexcept = default;
        constexpr IntIterator2WithStep& operator=(IntIterator2WithStep&&) noexcept = default;

        constexpr IntIterator2WithStep operator++(int) const
        {
            IntIterator2WithStep i = *this;
            operator++();
            return i;
        }
        constexpr IntIterator2WithStep operator--(int) const
        {
            IntIterator2WithStep i = *this;
            operator--();
            return i;
        }

        constexpr IntIterator2WithStep& operator++()
        {
            m_coords.x += m_step.x;
            if (m_coords.x > m_maxX)
            {
                m_coords.y += m_step.y;
                m_coords.x = m_minX;
            };
            return *this;
        }
        constexpr IntIterator2WithStep& operator--()
        {
            m_coords.x += m_step.x;
            if (m_coords.x < m_minX)
            {
                m_coords.y += m_step.y;
                m_coords.x = m_maxX;
            };
            return *this;
        }

        [[nodiscard]] constexpr IntIterator2WithStep operator+(difference_type n) const
        {
            IntIterator2WithStep i = *this;
            i += n;
            return i;
        }
        [[nodiscard]] constexpr IntIterator2WithStep operator-(difference_type n) const
        {
            IntIterator2WithStep i = *this;
            i -= n;
            return i;
        }

        constexpr IntIterator2WithStep& operator+=(difference_type n)
        {
            const IntType width = (m_maxX - m_minX + 1);
            const IntType widthSteps = (m_maxX - m_minX + 1) / m_step.x;
            m_coords.y += n / widthSteps * m_step.y;
            m_coords.x += n % widthSteps * m_step.x;
            if (m_coords.x > m_maxX)
            {
                m_coords.y += m_step.y;
                m_coords.x -= width;
            }
            return *this;
        }
        constexpr IntIterator2WithStep& operator-=(difference_type n)
        {
            const IntType width = (m_maxX - m_minX + 1);
            const IntType widthSteps = (m_maxX - m_minX + 1) / m_step.x;
            m_coords.y -= n / widthSteps * m_step.y;
            m_coords.x -= n % widthSteps * m_step.x;
            if (m_coords.x < m_minX)
            {
                m_coords.y -= m_step.y;
                m_coords.x += width * m_step.x;
            }
            return *this;
        }

        [[nodiscard]] constexpr difference_type operator-(const IntIterator2WithStep& rhs) const
        {
            const IntType width = (m_maxX - m_minX + 1) / m_step.x;
            const IntType x = (m_coords.x - rhs.m_coords.x) / m_step.x;
            const IntType y = (m_coords.y - rhs.m_coords.y) / m_step.y;
            return (y * width + x);
        }

        [[nodiscard]] constexpr value_type operator[](difference_type n) const
        {
            return m_coords + n;
        }
        [[nodiscard]] constexpr value_type operator*() const
        {
            return m_coords;
        }

        [[nodiscard]] constexpr bool operator==(const IntIterator2WithStep& rhs) const noexcept
        {
            return m_coords.x == rhs.m_coords.x && m_coords.y == rhs.m_coords.y;
        }
        [[nodiscard]] constexpr bool operator<(const IntIterator2WithStep& rhs) const noexcept
        {
            return *this - rhs < 0;
        }
        [[nodiscard]] constexpr bool operator>(const IntIterator2WithStep& rhs) const noexcept
        {
            return *this - rhs > 0;
        }
        [[nodiscard]] constexpr bool operator<=(const IntIterator2WithStep& rhs) const noexcept
        {
            return *this - rhs <= 0;
        }
        [[nodiscard]] constexpr bool operator>=(const IntIterator2WithStep& rhs) const noexcept
        {
            return *this - rhs >= 0;
        }
        [[nodiscard]] constexpr bool operator!=(const IntIterator2WithStep& rhs) const noexcept
        {
            return !operator==(rhs);
        }

    private:
        PointType m_coords;
        IntType m_minX;
        IntType m_maxX;
        VecType m_step;
    };

    template <typename IntT, bool HasStepV = false>
    struct IntRange2
    {
        static_assert(std::is_integral_v<IntT> && std::is_signed_v<IntT>, "Must be based on a signed integer");
        using IntType = IntT;
        using PointType = Point2<IntType>;

        using iterator = IntIterator2<IntT>;
        using const_iterator = IntIterator2<IntT>;

        constexpr IntRange2(const PointType& end) noexcept :
            m_begin(0, 0),
            m_end(end)
        {
        }

        constexpr IntRange2(const PointType& begin, const PointType& end) noexcept :
            m_begin(begin),
            m_end(end)
        {
        }

        [[nodiscard]] constexpr iterator begin()
        {
            return const_iterator(m_begin, m_begin.x, m_end.x - 1);
        }

        [[nodiscard]] constexpr const_iterator begin() const
        {
            return const_iterator(m_begin, m_begin.x, m_end.x - 1);
        }

        [[nodiscard]] constexpr const_iterator cbegin() const
        {
            return const_iterator(m_begin, m_begin.x, m_end.x - 1);
        }

        [[nodiscard]] constexpr iterator end()
        {
            return const_iterator(PointType(m_begin.x, m_end.y), m_begin.x, m_end.x - 1);
        }

        [[nodiscard]] constexpr const_iterator end() const
        {
            return const_iterator(PointType(m_begin.x, m_end.y), m_begin.x, m_end.x - 1);
        }

        [[nodiscard]] constexpr const_iterator cend() const
        {
            return const_iterator(PointType(m_begin.x, m_end.y), m_begin.x, m_end.x - 1);
        }

    private:
        PointType m_begin;
        PointType m_end;
    };

    template <typename IntT>
    struct IntRange2<IntT, true>
    {
        static_assert(std::is_integral_v<IntT> && std::is_signed_v<IntT>, "Must be based on a signed integer");
        using IntType = IntT;
        using VecType = Vec2<IntType>;
        using PointType = Point2<IntType>;

        using iterator = IntIterator2WithStep<IntT>;
        using const_iterator = IntIterator2WithStep<IntT>;

        constexpr IntRange2(const PointType& begin, const PointType& end, const VecType& step) noexcept :
            m_begin(begin),
            m_end(adjustedEnd(begin, end, step)),
            m_step(step)
        {
        }

        [[nodiscard]] constexpr iterator begin()
        {
            return const_iterator(m_begin, m_begin.x, m_end.x - 1, m_step);
        }

        [[nodiscard]] constexpr const_iterator begin() const
        {
            return const_iterator(m_begin, m_begin.x, m_end.x - 1, m_step);
        }

        [[nodiscard]] constexpr const_iterator cbegin() const
        {
            return const_iterator(m_begin, m_begin.x, m_end.x - 1, m_step);
        }

        [[nodiscard]] constexpr iterator end()
        {
            return const_iterator(PointType(m_begin.x, m_end.y), m_begin.x, m_end.x - 1, m_step);
        }

        [[nodiscard]] constexpr const_iterator end() const
        {
            return const_iterator(PointType(m_begin.x, m_end.y), m_begin.x, m_end.x - 1, m_step);
        }

        [[nodiscard]] constexpr const_iterator cend() const
        {
            return const_iterator(PointType(m_begin.x, m_end.y), m_begin.x, m_end.x - 1, m_step);
        }

    private:
        PointType m_begin;
        PointType m_end;
        VecType m_step;

        [[nodiscard]] constexpr PointType adjustedEnd(const PointType& begin, const PointType& end, const VecType& step)
        {
            return begin + ((end - begin + step - VecType(1, 1)) / step * step);
        }
    };

    template<typename IntT, bool HasStepV = false> IntRange2(const Point2<IntT>&)->IntRange2<IntT, false>;
    template<typename IntT, bool HasStepV = false> IntRange2(const Point2<IntT>&, const Point2<IntT>&)->IntRange2<IntT, false>;
    template<typename IntT, bool HasStepV = false> IntRange2(const Point2<IntT>&, const Point2<IntT>&, const Vec2<IntT>&)->IntRange2<IntT, true>;

    static_assert(std::is_same_v<decltype(IntRange2(Point2i(1, 1))), IntRange2<int, false>>);
    static_assert(std::is_same_v<decltype(IntRange2(Point2i(0, 0), Point2i(1, 1))), IntRange2<int, false>>);
    static_assert(std::is_same_v<decltype(IntRange2(Point2i(0, 0), Point2i(1, 1), Vec2i(2, 2))), IntRange2<int, true>>);
}
