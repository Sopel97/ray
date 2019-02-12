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
    struct Normal2
    {
        detail::OwnedReadOnlyProperty<T, Normal2<T>> x, y;

        constexpr static Normal2<T> xAxis()
        {
            return Normal2<T>(AssumeNormalized{}, 1, 0);
        }

        constexpr static Normal2<T> yAxis()
        {
            return Normal2<T>(AssumeNormalized{}, 0, 1);
        }

        constexpr Normal2() :
            x(1),
            y(0)
        {
        }

        constexpr Normal2(const T& x, const T& y) :
            Normal2(Vec2<T>(x, y).normalized())
        {
        }

        constexpr explicit Normal2(const Vec2<T>& v) :
            Normal2(v.normalized())
        {
        }

        constexpr Normal2(AssumeNormalized, const T& x, const T& y) :
            x(x),
            y(y)
        {
        }

        constexpr Normal2(AssumeNormalized, const Vec2<T>& v) :
            x(v.x),
            y(v.y)
        {
        }

        constexpr Normal2(const Normal2<T>&) = default;
        constexpr Normal2(Normal2<T>&&) noexcept = default;

        constexpr Normal2<T>& operator=(const Normal2<T>&) = default;
        constexpr Normal2<T>& operator=(Normal2<T>&&) noexcept = default;

        constexpr explicit operator Vec2<T>() const
        {
            return Vec2<T>(x, y);
        }

        constexpr Vec2<T> reciprocal() const
        {
            return Vec2<T>(
                static_cast<T>(1) / x,
                static_cast<T>(1) / y
                );
        }
    };

    using Normal2f = Normal2<float>;

    template <typename T>
    struct Vec2
    {
        T x, y;

        constexpr Vec2() = default;
        constexpr Vec2(const T& x, const T& y) :
            x(x),
            y(y)
        {

        }
        constexpr Vec2(const Vec2<T>&) = default;
        constexpr Vec2(Vec2<T>&&) noexcept = default;

        constexpr Vec2<T>& operator=(const Vec2<T>&) = default;
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

        constexpr T length() const
        {
            using std::sqrt;
            return sqrt(lengthSqr());
        }

        constexpr T lengthSqr() const
        {
            return
                x * x +
                y * y;
        }

        constexpr T invLength() const
        {
            return static_cast<T>(1) / length();
        }

        constexpr Vec2<T> reciprocal() const
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

        constexpr Normal2<T> assumeNormalized() const
        {
            return Normal2<T>(AssumeNormalized{}, *this);
        }

        constexpr Normal2<T> normalized() const
        {
            return Normal2<T>(AssumeNormalized{}, *this * invLength());
        }

        constexpr T max() const
        {
            using std::max;
            return max(x, y);
        }

        constexpr T min() const
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

        constexpr static Point2<T> origin()
        {
            return Point2<T>(0, 0);
        }

        constexpr Point2() = default;
        constexpr Point2(const T& x, const T& y) :
            x(x),
            y(y)
        {

        }
        constexpr Point2(const Point2<T>&) = default;
        constexpr Point2(Point2<T>&&) noexcept = default;

        constexpr Point2<T>& operator=(const Point2<T>&) = default;
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

        constexpr explicit operator Vec2<T>() const
        {
            return Vec2<T>(x, y);
        }
    };

    using Point2f = Point2<float>;
    using Point2i = Point2<int>;

    // Operations

    template <typename T>
    constexpr Vec2<T> operator-(const Point2<T>& lhs, const Point2<T>& rhs)
    {
        return Vec2<T>(
            lhs.x - rhs.x,
            lhs.y - rhs.y
            );
    }

    template <typename T>
    constexpr Point2<T> operator-(const Point2<T>& lhs, const Vec2<T>& rhs)
    {
        return Point2<T>(
            lhs.x - rhs.x,
            lhs.y - rhs.y
            );
    }

    template <typename T>
    constexpr Point2<T> operator+(const Point2<T>& lhs, const Vec2<T>& rhs)
    {
        return Point2<T>(
            lhs.x + rhs.x,
            lhs.y + rhs.y
            );
    }

    template <typename T>
    constexpr Vec2<T> operator+(const Vec2<T>& lhs, const Vec2<T>& rhs)
    {
        return Vec2<T>(
            lhs.x + rhs.x,
            lhs.y + rhs.y
            );
    }

    template <typename T>
    constexpr Vec2<T> operator-(const Vec2<T>& lhs, const Vec2<T>& rhs)
    {
        return Vec2<T>(
            lhs.x - rhs.x,
            lhs.y - rhs.y
            );
    }

    template <typename T>
    constexpr Vec2<T> operator*(const Vec2<T>& lhs, const Vec2<T>& rhs)
    {
        return Vec2<T>(
            lhs.x * rhs.x,
            lhs.y * rhs.y
            );
    }

    template <typename T>
    constexpr Vec2<T> operator/(const Vec2<T>& lhs, const Vec2<T>& rhs)
    {
        return Vec2<T>(
            lhs.x / rhs.x,
            lhs.y / rhs.y
            );
    }

    template <typename T>
    constexpr Vec2<T> operator*(const Vec2<T>& lhs, const T& rhs)
    {
        return Vec2<T>(
            lhs.x * rhs,
            lhs.y * rhs
            );
    }

    template <typename T>
    constexpr Vec2<T> operator*(const Normal2<T>& lhs, const T& rhs)
    {
        return Vec2<T>(
            lhs.x * rhs,
            lhs.y * rhs
            );
    }

    template <typename T>
    constexpr Vec2<T> operator*(const T& lhs, const Vec2<T>& rhs)
    {
        return Vec2<T>(
            lhs * rhs.x,
            lhs * rhs.y
            );
    }

    template <typename T>
    constexpr Vec2<T> operator/(const Vec2<T>& lhs, const T& rhs)
    {
        return Vec2<T>(
            lhs.x / rhs,
            lhs.y / rhs
            );
    }

    template <typename T>
    constexpr Vec2<T> operator*(const T& lhs, const Normal2<T>& rhs)
    {
        return Vec2<T>(
            lhs * rhs.x,
            lhs * rhs.y
            );
    }

    template <typename T>
    constexpr Vec2<T> operator/(const Normal2<T>& lhs, const T& rhs)
    {
        return Vec2<T>(
            lhs.x / rhs,
            lhs.y / rhs
            );
    }

    template <typename T>
    constexpr T dot(const Vec2<T>& lhs, const Vec2<T>& rhs)
    {
        return
            lhs.x * rhs.x +
            lhs.y * rhs.y;
    }

    template <typename T>
    constexpr T dot(const Normal2<T>& lhs, const Vec2<T>& rhs)
    {
        return
            lhs.x * rhs.x +
            lhs.y * rhs.y;
    }

    template <typename T>
    constexpr T dot(const Vec2<T>& lhs, const Normal2<T>& rhs)
    {
        return
            lhs.x * rhs.x +
            lhs.y * rhs.y;
    }

    template <typename T>
    constexpr T dot(const Normal2<T>& lhs, const Normal2<T>& rhs)
    {
        return
            lhs.x * rhs.x +
            lhs.y * rhs.y;
    }

    template <typename T>
    constexpr T cross(const Vec2<T>& lhs, const Vec2<T>& rhs)
    {
        return 
            lhs.x * rhs.y - 
            lhs.y * rhs.x;
    }

    template <typename T>
    constexpr Vec2<T> cross(const Normal2<T>& lhs, const Vec2<T>& rhs)
    {
        return
            lhs.x * rhs.y -
            lhs.y * rhs.x;
    }

    template <typename T>
    constexpr Vec2<T> cross(const Vec2<T>& lhs, const Normal2<T>& rhs)
    {
        return
            lhs.x * rhs.y -
            lhs.y * rhs.x;
    }

    template <typename T>
    constexpr Vec2<T> cross(const Normal2<T>& lhs, const Normal2<T>& rhs)
    {
        return
            lhs.x * rhs.y -
            lhs.y * rhs.x;
    }

    template <typename T>
    constexpr Vec2<T> min(const Vec2<T>& lhs, const Vec2<T>& rhs)
    {
        using std::min;
        return Vec2<T>(
            min(lhs.x, rhs.x),
            min(lhs.y, rhs.y)
            );
    }

    template <typename T>
    constexpr Vec2<T> max(const Vec2<T>& lhs, const Vec2<T>& rhs)
    {
        using std::max;
        return Vec2<T>(
            max(lhs.x, rhs.x),
            max(lhs.y, rhs.y)
            );
    }

    template <typename T>
    constexpr T distance(const Point2<T>& lhs, const Point2<T>& rhs)
    {
        return (rhs - lhs).length();
    }

    template <typename T>
    constexpr T distanceSqr(const Point2<T>& lhs, const Point2<T>& rhs)
    {
        return (rhs - lhs).lengthSqr();
    }

    // projection of v on n
    template <typename T>
    constexpr Vec2<T> projection(const Vec2<T>& v, const Vec2<T>& n)
    {
        return n * (dot(v, n) / dot(n, n));
    }

    // projection of v on n
    template <typename T>
    constexpr Vec2<T> projection(const Vec2<T>& v, const Normal2<T>& n)
    {
        return n * dot(v, n);
    }

    // projection of v on n
    template <typename T>
    constexpr Vec2<T> projection(const Normal2<T>& v, const Vec2<T>& n)
    {
        return n * (dot(v, n) / dot(n, n));
    }

    // projection of v on n
    template <typename T>
    constexpr Vec2<T> projection(const Normal2<T>& v, const Normal2<T>& n)
    {
        return n * dot(v, n);
    }

    // reflection of v from a surface with normal n (like light reflection)
    template <typename T>
    constexpr Vec2<T> reflection(const Vec2<T>& v, const Vec2<T>& n)
    {
        return v - static_cast<T>(2) * projection(v, n);
    }

    // reflection of v from a surface with normal n (like light reflection)
    template <typename T>
    constexpr Vec2<T> reflection(const Vec2<T>& v, const Normal2<T>& n)
    {
        return v - static_cast<T>(2) * projection(v, n);
    }

    // reflection of v from a surface with normal n (like light reflection)
    template <typename T>
    constexpr Normal2<T> reflection(const Normal2<T>& v, const Vec2<T>& n)
    {
        return (Vec2<T>(v) - static_cast<T>(2) * projection(v, n)).assumeNormalized();
    }

    // reflection of v from a surface with normal n (like light reflection)
    template <typename T>
    constexpr Normal2<T> reflection(const Normal2<T>& v, const Normal2<T>& n)
    {
        return (Vec2<T>(v) - static_cast<T>(2) * projection(v, n)).assumeNormalized();
    }

    // refraction of v through a surface with normal n with refractive index ratio of r
    template <typename T>
    constexpr Vec2<T> refraction(const Vec2<T>& v, const Vec2<T>& n, const T& r)
    {
        using std::sqrt;

        const T c = -dot(v, n);
        const T s = sqrt(static_cast<T>(1) - r * r*(static_cast<T>(1) - c * c));
        return r * v + (r*c - s)*n;
    }

    // refraction of v through a surface with normal n with refractive index ratio of r
    template <typename T>
    constexpr Normal2<T> refraction(const Normal2<T>& v, const Vec2<T>& n, const T& r)
    {
        using std::sqrt;

        const T c = -dot(v, n);
        const T s = sqrt(static_cast<T>(1) - r * r*(static_cast<T>(1) - c * c));
        return (r * v + (r*c - s)*n).normalized();
    }

    // refraction of v through a surface with normal n with refractive index ratio of r
    template <typename T>
    constexpr Vec2<T> refraction(const Vec2<T>& v, const Normal2<T>& n, const T& r)
    {
        using std::sqrt;

        const T c = -dot(v, n);
        const T s = sqrt(static_cast<T>(1) - r * r*(static_cast<T>(1) - c * c));
        return r * v + (r*c - s)*n;
    }

    // refraction of v through a surface with normal n with refractive index ratio of r
    template <typename T>
    constexpr Normal2<T> refraction(const Normal2<T>& v, const Normal2<T>& n, const T& r)
    {
        using std::sqrt;

        const T c = -dot(v, n);
        const T s = sqrt(static_cast<T>(1) - r * r*(static_cast<T>(1) - c * c));
        return (r * v + (r*c - s)*n).assumeNormalized();
    }

    template <typename T>
    constexpr Normal2<T> operator-(const Normal2<T>& v)
    {
        return Normal2<T>(AssumeNormalized{}, -v.x, -v.y);
    }

    template <typename T>
    constexpr Vec2<T> operator-(const Vec2<T>& v)
    {
        return Vec2<T>(-v.x, -v.y);
    }

    template <typename T>
    constexpr Point2<T> operator-(const Point2<T>& v)
    {
        return Point2<T>(-v.x, -v.y);
    }
}
