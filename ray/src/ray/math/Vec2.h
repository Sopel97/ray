#pragma once

#include "detail/VecDetail.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace ray
{
    template <typename T>
    struct Vec2;

    template <typename T>
    struct UnitVec2
    {
        detail::OwnedReadOnlyProperty<T, UnitVec2<T>> x, y;

        [[nodiscard]] constexpr static UnitVec2<T> xAxis() noexcept
        {
            return UnitVec2<T>(AssumeNormalized{}, 1, 0);
        }

        [[nodiscard]] constexpr static UnitVec2<T> yAxis() noexcept
        {
            return UnitVec2<T>(AssumeNormalized{}, 0, 1);
        }

        constexpr UnitVec2() noexcept :
            x(1),
            y(0)
        {
        }

        constexpr UnitVec2(const T& x, const T& y) noexcept :
            UnitVec2(Vec2<T>(x, y).normalized())
        {
        }

        constexpr explicit UnitVec2(const Vec2<T>& v) noexcept :
            UnitVec2(v.normalized())
        {
        }

        constexpr UnitVec2(AssumeNormalized, const T& x, const T& y) noexcept :
            x(x),
            y(y)
        {
        }

        constexpr UnitVec2(AssumeNormalized, const Vec2<T>& v) noexcept :
            x(v.x),
            y(v.y)
        {
        }

        constexpr UnitVec2(const UnitVec2<T>&) noexcept = default;
        constexpr UnitVec2(UnitVec2<T>&&) noexcept = default;

        constexpr UnitVec2<T>& operator=(const UnitVec2<T>&) noexcept = default;
        constexpr UnitVec2<T>& operator=(UnitVec2<T>&&) noexcept = default;

        [[nodiscard]] constexpr explicit operator Vec2<T>() const
        {
            return Vec2<T>(x, y);
        }

        [[nodiscard]] constexpr Vec2<T> reciprocal() const
        {
            return Vec2<T>(
                static_cast<T>(1) / x,
                static_cast<T>(1) / y
                );
        }
    };

    using UnitVec2f = UnitVec2<float>;

    template <typename T>
    struct Normal2 : UnitVec2<T>
    {
        using UnitVec2<T>::UnitVec2;
    };

    using Normal2f = Normal2<float>;

    template <typename T>
    struct Vec2
    {
        T x, y;

        [[nodiscard]] constexpr static Vec2<T> broadcast(const T& v) noexcept
        {
            return Vec2<T>(v, v);
        }

        constexpr Vec2() noexcept = default;
        constexpr Vec2(const T& x, const T& y) noexcept :
            x(x),
            y(y)
        {

        }
        constexpr Vec2(const Vec2<T>&) noexcept = default;
        constexpr Vec2(Vec2<T>&&) noexcept = default;

        constexpr Vec2<T>& operator=(const Vec2<T>&) noexcept = default;
        constexpr Vec2<T>& operator=(Vec2<T>&&) noexcept = default;

        constexpr Vec2<T>& operator+=(const Vec2<T>& rhs)
        {
            x += rhs.x;
            y += rhs.y;
            return *this;
        }

        constexpr Vec2<T>& operator-=(const Vec2<T>& rhs)
        {
            x -= rhs.x;
            y -= rhs.y;
            return *this;
        }

        constexpr Vec2<T>& operator*=(const Vec2<T>& rhs)
        {
            x *= rhs.x;
            y *= rhs.y;
            return *this;
        }

        constexpr Vec2<T>& operator/=(const Vec2<T>& rhs)
        {
            x /= rhs.x;
            y /= rhs.y;
            return *this;
        }

        constexpr Vec2<T>& operator*=(const T& rhs)
        {
            x *= rhs;
            y *= rhs;
            return *this;
        }

        constexpr Vec2<T>& operator/=(const T& rhs)
        {
            x /= rhs;
            y /= rhs;
            return *this;
        }

        [[nodiscard]] constexpr T length() const
        {
            using std::sqrt;
            return sqrt(lengthSqr());
        }

        [[nodiscard]] constexpr T lengthSqr() const
        {
            return
                x * x +
                y * y;
        }

        [[nodiscard]] constexpr T invLength() const
        {
            return static_cast<T>(1) / length();
        }

        [[nodiscard]] constexpr Vec2<T> reciprocal() const
        {
            return Vec2<T>(
                static_cast<T>(1) / x,
                static_cast<T>(1) / y
                );
        }

        constexpr void normalize()
        {
            *this *= invLength();
        }

        [[nodiscard]] constexpr UnitVec2<T> assumeNormalized() const
        {
            return UnitVec2<T>(AssumeNormalized{}, *this);
        }

        [[nodiscard]] constexpr UnitVec2<T> normalized() const
        {
            return UnitVec2<T>(AssumeNormalized{}, *this * invLength());
        }

        [[nodiscard]] constexpr T max() const
        {
            using std::max;
            return max(x, y);
        }

        [[nodiscard]] constexpr T min() const
        {
            using std::min;
            return min(x, y);
        }
    };

    using Vec2f = Vec2<float>;
    using Vec2i = Vec2<int>;

    template <typename T>
    struct Point2
    {
        T x, y;

        [[nodiscard]] constexpr static Point2<T> broadcast(const T& v) noexcept
        {
            return Point2<T>(v, v);
        }

        [[nodiscard]] constexpr static Point2<T> origin() noexcept 
        {
            return Point2<T>(0, 0);
        }

        constexpr Point2() noexcept = default;
        constexpr Point2(const T& x, const T& y) noexcept :
            x(x),
            y(y)
        {

        }
        constexpr Point2(const Point2<T>&) noexcept = default;
        constexpr Point2(Point2<T>&&) noexcept = default;

        constexpr Point2<T>& operator=(const Point2<T>&) noexcept = default;
        constexpr Point2<T>& operator=(Point2<T>&&) noexcept = default;

        constexpr Point2<T>& operator+=(const Vec2<T>& rhs)
        {
            x += rhs.x;
            y += rhs.y;
            return *this;
        }

        constexpr Point2<T>& operator-=(const Vec2<T>& rhs)
        {
            x -= rhs.x;
            y -= rhs.y;
            return *this;
        }

        [[nodiscard]] constexpr explicit operator Vec2<T>() const
        {
            return Vec2<T>(x, y);
        }
    };

    using Point2f = Point2<float>;
    using Point2i = Point2<int>;

    // Operations

    template <typename T>
    [[nodiscard]] constexpr Vec2<T> operator-(const Point2<T>& lhs, const Point2<T>& rhs)
    {
        return Vec2<T>(
            lhs.x - rhs.x,
            lhs.y - rhs.y
            );
    }

    template <typename T>
    [[nodiscard]] constexpr Point2<T> operator-(const Point2<T>& lhs, const Vec2<T>& rhs)
    {
        return Point2<T>(
            lhs.x - rhs.x,
            lhs.y - rhs.y
            );
    }

    template <typename T>
    [[nodiscard]] constexpr Point2<T> operator+(const Point2<T>& lhs, const Vec2<T>& rhs)
    {
        return Point2<T>(
            lhs.x + rhs.x,
            lhs.y + rhs.y
            );
    }

    template <typename T>
    [[nodiscard]] constexpr Vec2<T> operator+(const Vec2<T>& lhs, const Vec2<T>& rhs)
    {
        return Vec2<T>(
            lhs.x + rhs.x,
            lhs.y + rhs.y
            );
    }

    template <typename T>
    [[nodiscard]] constexpr Vec2<T> operator-(const Vec2<T>& lhs, const Vec2<T>& rhs)
    {
        return Vec2<T>(
            lhs.x - rhs.x,
            lhs.y - rhs.y
            );
    }

    template <typename T>
    [[nodiscard]] constexpr Vec2<T> operator*(const Vec2<T>& lhs, const Vec2<T>& rhs)
    {
        return Vec2<T>(
            lhs.x * rhs.x,
            lhs.y * rhs.y
            );
    }

    template <typename T>
    [[nodiscard]] constexpr Vec2<T> operator/(const Vec2<T>& lhs, const Vec2<T>& rhs)
    {
        return Vec2<T>(
            lhs.x / rhs.x,
            lhs.y / rhs.y
            );
    }

    template <typename T>
    [[nodiscard]] constexpr Vec2<T> operator*(const Vec2<T>& lhs, const T& rhs)
    {
        return Vec2<T>(
            lhs.x * rhs,
            lhs.y * rhs
            );
    }

    template <typename T>
    [[nodiscard]] constexpr Vec2<T> operator*(const UnitVec2<T>& lhs, const T& rhs)
    {
        return Vec2<T>(
            lhs.x * rhs,
            lhs.y * rhs
            );
    }

    template <typename T>
    [[nodiscard]] constexpr Vec2<T> operator*(const T& lhs, const Vec2<T>& rhs)
    {
        return Vec2<T>(
            lhs * rhs.x,
            lhs * rhs.y
            );
    }

    template <typename T>
    [[nodiscard]] constexpr Vec2<T> operator/(const Vec2<T>& lhs, const T& rhs)
    {
        return Vec2<T>(
            lhs.x / rhs,
            lhs.y / rhs
            );
    }

    template <typename T>
    [[nodiscard]] constexpr Vec2<T> operator*(const T& lhs, const UnitVec2<T>& rhs)
    {
        return Vec2<T>(
            lhs * rhs.x,
            lhs * rhs.y
            );
    }

    template <typename T>
    [[nodiscard]] constexpr Vec2<T> operator/(const UnitVec2<T>& lhs, const T& rhs)
    {
        return Vec2<T>(
            lhs.x / rhs,
            lhs.y / rhs
            );
    }

    template <typename T>
    [[nodiscard]] constexpr T dot(const Vec2<T>& lhs, const Vec2<T>& rhs)
    {
        return
            lhs.x * rhs.x +
            lhs.y * rhs.y;
    }

    template <typename T>
    [[nodiscard]] constexpr T dot(const UnitVec2<T>& lhs, const Vec2<T>& rhs)
    {
        return
            lhs.x * rhs.x +
            lhs.y * rhs.y;
    }

    template <typename T>
    [[nodiscard]] constexpr T dot(const Vec2<T>& lhs, const UnitVec2<T>& rhs)
    {
        return
            lhs.x * rhs.x +
            lhs.y * rhs.y;
    }

    template <typename T>
    [[nodiscard]] constexpr T dot(const UnitVec2<T>& lhs, const UnitVec2<T>& rhs)
    {
        return
            lhs.x * rhs.x +
            lhs.y * rhs.y;
    }

    template <typename T>
    [[nodiscard]] constexpr T cross(const Vec2<T>& lhs, const Vec2<T>& rhs)
    {
        return 
            lhs.x * rhs.y - 
            lhs.y * rhs.x;
    }

    template <typename T>
    [[nodiscard]] constexpr Vec2<T> cross(const UnitVec2<T>& lhs, const Vec2<T>& rhs)
    {
        return
            lhs.x * rhs.y -
            lhs.y * rhs.x;
    }

    template <typename T>
    [[nodiscard]] constexpr Vec2<T> cross(const Vec2<T>& lhs, const UnitVec2<T>& rhs)
    {
        return
            lhs.x * rhs.y -
            lhs.y * rhs.x;
    }

    template <typename T>
    [[nodiscard]] constexpr Vec2<T> cross(const UnitVec2<T>& lhs, const UnitVec2<T>& rhs)
    {
        return
            lhs.x * rhs.y -
            lhs.y * rhs.x;
    }

    template <typename T>
    [[nodiscard]] constexpr Vec2<T> min(const Vec2<T>& lhs, const Vec2<T>& rhs)
    {
        using std::min;
        return Vec2<T>(
            min(lhs.x, rhs.x),
            min(lhs.y, rhs.y)
            );
    }

    template <typename T>
    [[nodiscard]] constexpr Vec2<T> max(const Vec2<T>& lhs, const Vec2<T>& rhs)
    {
        using std::max;
        return Vec2<T>(
            max(lhs.x, rhs.x),
            max(lhs.y, rhs.y)
            );
    }

    template <typename T>
    [[nodiscard]] constexpr T distance(const Point2<T>& lhs, const Point2<T>& rhs)
    {
        return (rhs - lhs).length();
    }

    template <typename T>
    [[nodiscard]] constexpr T distanceSqr(const Point2<T>& lhs, const Point2<T>& rhs)
    {
        return (rhs - lhs).lengthSqr();
    }

    // projection of v on n
    template <typename T>
    [[nodiscard]] constexpr Vec2<T> projection(const Vec2<T>& v, const Vec2<T>& n)
    {
        return n * (dot(v, n) / dot(n, n));
    }

    // projection of v on n
    template <typename T>
    [[nodiscard]] constexpr Vec2<T> projection(const Vec2<T>& v, const UnitVec2<T>& n)
    {
        return n * dot(v, n);
    }

    // projection of v on n
    template <typename T>
    [[nodiscard]] constexpr Vec2<T> projection(const UnitVec2<T>& v, const Vec2<T>& n)
    {
        return n * (dot(v, n) / dot(n, n));
    }

    // projection of v on n
    template <typename T>
    [[nodiscard]] constexpr Vec2<T> projection(const UnitVec2<T>& v, const UnitVec2<T>& n)
    {
        return n * dot(v, n);
    }

    // reflection of v from a surface with normal n (like light reflection)
    template <typename T>
    [[nodiscard]] constexpr Vec2<T> reflection(const Vec2<T>& v, const Vec2<T>& n)
    {
        return v - static_cast<T>(2) * projection(v, n);
    }

    // reflection of v from a surface with normal n (like light reflection)
    template <typename T>
    [[nodiscard]] constexpr Vec2<T> reflection(const Vec2<T>& v, const UnitVec2<T>& n)
    {
        return v - static_cast<T>(2) * projection(v, n);
    }

    // reflection of v from a surface with normal n (like light reflection)
    template <typename T>
    [[nodiscard]] constexpr UnitVec2<T> reflection(const UnitVec2<T>& v, const Vec2<T>& n)
    {
        return (Vec2<T>(v) - static_cast<T>(2) * projection(v, n)).assumeNormalized();
    }

    // reflection of v from a surface with normal n (like light reflection)
    template <typename T>
    [[nodiscard]] constexpr UnitVec2<T> reflection(const UnitVec2<T>& v, const UnitVec2<T>& n)
    {
        return (Vec2<T>(v) - static_cast<T>(2) * projection(v, n)).assumeNormalized();
    }

    // refraction of v through a surface with normal n with refractive index ratio of r
    template <typename T>
    [[nodiscard]] constexpr Vec2<T> refraction(const Vec2<T>& v, const Vec2<T>& n, const T& r)
    {
        using std::sqrt;

        const T c = -dot(v, n);
        const T s = sqrt(static_cast<T>(1) - r * r*(static_cast<T>(1) - c * c));
        return r * v + (r*c - s)*n;
    }

    // refraction of v through a surface with normal n with refractive index ratio of r
    template <typename T>
    [[nodiscard]] constexpr UnitVec2<T> refraction(const UnitVec2<T>& v, const Vec2<T>& n, const T& r)
    {
        using std::sqrt;

        const T c = -dot(v, n);
        const T s = sqrt(static_cast<T>(1) - r * r*(static_cast<T>(1) - c * c));
        return (r * v + (r*c - s)*n).normalized();
    }

    // refraction of v through a surface with normal n with refractive index ratio of r
    template <typename T>
    [[nodiscard]] constexpr Vec2<T> refraction(const Vec2<T>& v, const UnitVec2<T>& n, const T& r)
    {
        using std::sqrt;

        const T c = -dot(v, n);
        const T s = sqrt(static_cast<T>(1) - r * r*(static_cast<T>(1) - c * c));
        return r * v + (r*c - s)*n;
    }

    // refraction of v through a surface with normal n with refractive index ratio of r
    template <typename T>
    [[nodiscard]] constexpr UnitVec2<T> refraction(const UnitVec2<T>& v, const UnitVec2<T>& n, const T& r)
    {
        using std::sqrt;

        const T c = -dot(v, n);
        const T s = sqrt(static_cast<T>(1) - r * r*(static_cast<T>(1) - c * c));
        return (r * v + (r*c - s)*n).assumeNormalized();
    }

    template <typename T>
    [[nodiscard]] constexpr UnitVec2<T> operator-(const UnitVec2<T>& v)
    {
        return UnitVec2<T>(AssumeNormalized{}, -v.x, -v.y);
    }

    template <typename T>
    [[nodiscard]] constexpr Vec2<T> operator-(const Vec2<T>& v)
    {
        return Vec2<T>(-v.x, -v.y);
    }

    template <typename T>
    [[nodiscard]] constexpr Point2<T> operator-(const Point2<T>& v)
    {
        return Point2<T>(-v.x, -v.y);
    }
}
