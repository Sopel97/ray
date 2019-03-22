#pragma once

#include "detail/VecDetail.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace ray
{
    template <typename T>
    struct Vec3Mask
    {
        bool x, y, z;

        [[nodiscard]] constexpr std::uint8_t packed() const
        {
            return x | (y << 1) | (z << 2);
        }
    };

    template <typename T>
    struct Vec3;

    template <typename T>
    struct Normal3
    {
        detail::OwnedReadOnlyProperty<T, Normal3<T>> x, y, z;

        [[nodiscard]] constexpr static Normal3<T> xAxis() noexcept
        {
            return Normal3<T>(AssumeNormalized{}, 1, 0, 0);
        }

        [[nodiscard]] constexpr static Normal3<T> yAxis() noexcept
        {
            return Normal3<T>(AssumeNormalized{}, 0, 1, 0);
        }

        [[nodiscard]] constexpr static Normal3<T> zAxis() noexcept
        {
            return Normal3<T>(AssumeNormalized{}, 0, 0, 1);
        }

        constexpr Normal3() noexcept :
            x(1),
            y(0),
            z(0)
        {
        }

        constexpr Normal3(const T& x, const T& y, const T& z) noexcept :
            Normal3(Vec3<T>(x, y, z).normalized())
        {
        }

        constexpr Normal3(AssumeNormalized, const T& x, const T& y, const T& z) noexcept :
            x(x),
            y(y),
            z(z)
        {
        }

        constexpr Normal3(AssumeNormalized, const Vec3<T>& v) noexcept :
            x(v.x),
            y(v.y),
            z(v.z)
        {
        }

        constexpr Normal3(const Normal3<T>&) noexcept = default;
        constexpr Normal3(Normal3<T>&&) noexcept = default;

        constexpr Normal3<T>& operator=(const Normal3<T>&) noexcept = default;
        constexpr Normal3<T>& operator=(Normal3<T>&&) noexcept = default;

        [[nodiscard]] constexpr operator Vec3<T>() const
        {
            return Vec3<T>(x, y, z);
        }

        [[nodiscard]] constexpr Vec3<T> reciprocal() const
        {
            return Vec3<T>(
                static_cast<T>(1) / x,
                static_cast<T>(1) / y,
                static_cast<T>(1) / z
            );
        }
    };

    using Normal3f = Normal3<float>;


    template <typename T>
    struct Vec3
    {
        T x, y, z;

        [[nodiscard]] constexpr static Vec3<T> broadcast(const T& v) noexcept
        {
            return Vec3<T>(v, v, v);
        }
        constexpr Vec3() noexcept = default;
        constexpr Vec3(const T& v) :
            x(v),
            y(v),
            z(v)
        {

        }

        constexpr Vec3(const T& x, const T& y, const T& z) noexcept :
            x(x),
            y(y),
            z(z)
        {

        }
        constexpr Vec3(const Vec3<T>&) noexcept = default;
        constexpr Vec3(Vec3<T>&&) noexcept = default;

        constexpr Vec3<T>& operator=(const Vec3<T>&) noexcept = default;
        constexpr Vec3<T>& operator=(Vec3<T>&&) noexcept = default;

        constexpr Vec3<T>& operator+=(const Vec3<T>& rhs)
        {
            x += rhs.x;
            y += rhs.y;
            z += rhs.z;
            return *this;
        }

        constexpr Vec3<T>& operator-=(const Vec3<T>& rhs)
        {
            x -= rhs.x;
            y -= rhs.y;
            z -= rhs.z;
            return *this;
        }

        constexpr Vec3<T>& operator*=(const Vec3<T>& rhs)
        {
            x *= rhs.x;
            y *= rhs.y;
            z *= rhs.z;
            return *this;
        }

        constexpr Vec3<T>& operator/=(const Vec3<T>& rhs)
        {
            x /= rhs.x;
            y /= rhs.y;
            z /= rhs.z;
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
                y * y +
                z * z;
        }

        [[nodiscard]] constexpr T invLength() const
        {
            return static_cast<T>(1) / length();
        }

        [[nodiscard]] constexpr Vec3<T> reciprocal() const
        {
            return Vec3<T>(
                static_cast<T>(1) / x,
                static_cast<T>(1) / y,
                static_cast<T>(1) / z
            );
        }

        constexpr void normalize()
        {
            *this *= invLength();
        }

        [[nodiscard]] constexpr Normal3<T> assumeNormalized() const
        {
            return Normal3<T>(AssumeNormalized{}, *this);
        }

        [[nodiscard]] constexpr Normal3<T> normalized() const
        {
            return Normal3<T>(AssumeNormalized{}, *this * invLength());
        }

        [[nodiscard]] constexpr T max() const
        {
            using std::max;
            return max(max(x, y), z);
        }

        [[nodiscard]] constexpr T min() const
        {
            using std::min;
            return min(min(x, y), z);
        }
    };

    using Vec3f = Vec3<float>;

    template <typename T>
    struct Point3
    {
        T x, y, z;

        [[nodiscard]] constexpr static Point3<T> broadcast(const T& v) noexcept
        {
            return Point3<T>(v, v, v);
        }
        [[nodiscard]] constexpr static Point3<T> origin() noexcept
        {
            return Point3<T>(0, 0, 0);
        }

        constexpr Point3() noexcept = default;
        constexpr Point3(const T& v) :
            x(v),
            y(v),
            z(v)
        {
        }

        constexpr Point3(const T& x, const T& y, const T& z) noexcept :
            x(x),
            y(y),
            z(z)
        {

        }
        constexpr Point3(const Point3<T>&) noexcept = default;
        constexpr Point3(Point3<T>&&) noexcept = default;

        constexpr Point3<T>& operator=(const Point3<T>&) noexcept = default;
        constexpr Point3<T>& operator=(Point3<T>&&) noexcept = default;

        constexpr Point3<T>& operator+=(const Vec3<T>& rhs)
        {
            x += rhs.x;
            y += rhs.y;
            z += rhs.z;
            return *this;
        }

        constexpr Point3<T>& operator-=(const Vec3<T>& rhs)
        {
            x -= rhs.x;
            y -= rhs.y;
            z -= rhs.z;
            return *this;
        }

        [[nodiscard]] constexpr operator Vec3<T>() const
        {
            return Vec3<T>(x, y, z);
        }
    };

    using Point3f = Point3<float>;

    // Operations

    template <typename T>
    [[nodiscard]] constexpr Vec3<T> operator-(const Point3<T>& lhs, const Point3<T>& rhs)
    {
        return Vec3<T>(
            lhs.x - rhs.x,
            lhs.y - rhs.y,
            lhs.z - rhs.z
            );
    }

    template <typename T>
    [[nodiscard]] constexpr Point3<T> operator-(const Point3<T>& lhs, const Vec3<T>& rhs)
    {
        return Point3<T>(
            lhs.x - rhs.x,
            lhs.y - rhs.y,
            lhs.z - rhs.z
            );
    }

    template <typename T>
    [[nodiscard]] constexpr Point3<T> operator+(const Point3<T>& lhs, const Vec3<T>& rhs)
    {
        return Point3<T>(
            lhs.x + rhs.x,
            lhs.y + rhs.y,
            lhs.z + rhs.z
            );
    }

    template <typename T>
    [[nodiscard]] constexpr Vec3<T> operator+(const Vec3<T>& lhs, const Vec3<T>& rhs)
    {
        return Vec3<T>(
            lhs.x + rhs.x,
            lhs.y + rhs.y,
            lhs.z + rhs.z
        );
    }

    template <typename T>
    [[nodiscard]] constexpr Vec3<T> operator-(const Vec3<T>& lhs, const Vec3<T>& rhs)
    {
        return Vec3<T>(
            lhs.x - rhs.x,
            lhs.y - rhs.y,
            lhs.z - rhs.z
            );
    }

    template <typename T>
    [[nodiscard]] constexpr Vec3<T> operator*(const Vec3<T>& lhs, const Vec3<T>& rhs)
    {
        return Vec3<T>(
            lhs.x * rhs.x,
            lhs.y * rhs.y,
            lhs.z * rhs.z
            );
    }

    template <typename T>
    [[nodiscard]] constexpr Vec3<T> operator/(const Vec3<T>& lhs, const Vec3<T>& rhs)
    {
        return Vec3<T>(
            lhs.x / rhs.x,
            lhs.y / rhs.y,
            lhs.z / rhs.z
            );
    }

    template <typename T>
    [[nodiscard]] constexpr Vec3Mask<T> operator<(const Vec3<T>& lhs, const Vec3<T>& rhs) noexcept
    {
        return Vec3Mask<T>{
            lhs.x < rhs.x,
            lhs.y < rhs.y,
            lhs.z < rhs.z
        };
    }

    template <typename T>
    [[nodiscard]] constexpr T dot(const Vec3<T>& lhs, const Vec3<T>& rhs)
    {
        return
            lhs.x * rhs.x +
            lhs.y * rhs.y +
            lhs.z * rhs.z;
    }

    template <typename T>
    [[nodiscard]] constexpr Vec3<T> cross(const Vec3<T>& lhs, const Vec3<T>& rhs)
    {
        return Vec3<T>(
            lhs.y*rhs.z - lhs.z*rhs.y, 
            lhs.z*rhs.x - lhs.x*rhs.z, 
            lhs.x*rhs.y - lhs.y*rhs.x
        );
    }

    template <typename T>
    [[nodiscard]] constexpr Vec3<T> min(const Vec3<T>& lhs, const Vec3<T>& rhs)
    {
        using std::min;
        return Vec3<T>(
            min(lhs.x, rhs.x),
            min(lhs.y, rhs.y),
            min(lhs.z, rhs.z)
            );
    }

    template <typename T>
    [[nodiscard]] constexpr Vec3<T> max(const Vec3<T>& lhs, const Vec3<T>& rhs)
    {
        using std::max;
        return Vec3<T>(
            max(lhs.x, rhs.x),
            max(lhs.y, rhs.y),
            max(lhs.z, rhs.z)
            );
    }

    template <typename T>
    [[nodiscard]] constexpr T mixed(const Vec3<T>& a, const Vec3<T>& b, const Vec3<T>& c)
    {
        return dot(a, cross(b, c));
    }

    template <typename T>
    [[nodiscard]] constexpr T distance(const Point3<T>& lhs, const Point3<T>& rhs)
    {
        return (rhs - lhs).length();
    }

    template <typename T>
    [[nodiscard]] constexpr T distanceSqr(const Point3<T>& lhs, const Point3<T>& rhs)
    {
        return (rhs - lhs).lengthSqr();
    }

    // projection of v on n
    template <typename T>
    [[nodiscard]] constexpr Vec3<T> projection(const Vec3<T>& v, const Vec3<T>& n)
    {
        return n * (dot(v, n) / dot(n, n));
    }

    // projection of v on n
    template <typename T>
    [[nodiscard]] constexpr Vec3<T> projection(const Vec3<T>& v, const Normal3<T>& n)
    {
        return n * dot(v, n);
    }

    // projection of v on n
    template <typename T>
    [[nodiscard]] constexpr Vec3<T> projection(const Normal3<T>& v, const Vec3<T>& n)
    {
        return n * (dot(v, n) / dot(n, n));
    }

    // projection of v on n
    template <typename T>
    [[nodiscard]] constexpr Vec3<T> projection(const Normal3<T>& v, const Normal3<T>& n)
    {
        return n * dot(v, n);
    }

    // reflection of v from a surface with normal n (like light reflection)
    template <typename T>
    [[nodiscard]] constexpr Vec3<T> reflection(const Vec3<T>& v, const Vec3<T>& n)
    {
        return v - static_cast<T>(2) * projection(v, n);
    }

    // reflection of v from a surface with normal n (like light reflection)
    template <typename T>
    [[nodiscard]] constexpr Vec3<T> reflection(const Vec3<T>& v, const Normal3<T>& n)
    {
        return v - static_cast<T>(2) * projection(v, n);
    }

    // reflection of v from a surface with normal n (like light reflection)
    template <typename T>
    [[nodiscard]] constexpr Normal3<T> reflection(const Normal3<T>& v, const Vec3<T>& n)
    {
        return (Vec3<T>(v) - static_cast<T>(2) * projection(v, n)).assumeNormalized();
    }

    // reflection of v from a surface with normal n (like light reflection)
    template <typename T>
    [[nodiscard]] constexpr Normal3<T> reflection(const Normal3<T>& v, const Normal3<T>& n)
    {
        return (Vec3<T>(v) - static_cast<T>(2) * projection(v, n)).assumeNormalized();
    }

    // refraction of v through a surface with normal n with refractive index ratio of r
    template <typename T>
    [[nodiscard]] constexpr Vec3<T> refraction(const Vec3<T>& v, const Vec3<T>& n, const T& r)
    {
        using std::sqrt;

        const T c = -dot(v, n);
        const T s = sqrt(static_cast<T>(1) - r * r*(static_cast<T>(1) - c * c));
        return r * v + (r*c - s)*n;
    }

    // refraction of v through a surface with normal n with refractive index ratio of r
    template <typename T>
    [[nodiscard]] constexpr Normal3<T> refraction(const Normal3<T>& v, const Vec3<T>& n, const T& r)
    {
        using std::sqrt;

        const T c = -dot(v, n);
        const T s = sqrt(static_cast<T>(1) - r * r*(static_cast<T>(1) - c * c));
        return (r * v + (r*c - s)*n).normalized();
    }

    // refraction of v through a surface with normal n with refractive index ratio of r
    template <typename T>
    [[nodiscard]] constexpr Vec3<T> refraction(const Vec3<T>& v, const Normal3<T>& n, const T& r)
    {
        using std::sqrt;

        const T c = -dot(v, n);
        const T s = sqrt(static_cast<T>(1) - r * r*(static_cast<T>(1) - c * c));
        return r * v + (r*c - s)*n;
    }

    // refraction of v through a surface with normal n with refractive index ratio of r
    template <typename T>
    [[nodiscard]] constexpr Normal3<T> refraction(const Normal3<T>& v, const Normal3<T>& n, const T& r)
    {
        using std::sqrt;

        const T c = -dot(v, n);
        const T s = sqrt(static_cast<T>(1) - r * r*(static_cast<T>(1) - c * c));
        return (r * v + (r*c - s)*n).assumeNormalized();
    }

    template <typename T>
    [[nodiscard]] constexpr Normal3<T> operator-(const Normal3<T>& v)
    {
        return Normal3<T>(AssumeNormalized{}, -v.x, -v.y, -v.z);
    }

    template <typename T>
    [[nodiscard]] constexpr Vec3<T> operator-(const Vec3<T>& v)
    {
        return Vec3<T>(-v.x, -v.y, -v.z);
    }

    template <typename T>
    [[nodiscard]] constexpr Point3<T> operator-(const Point3<T>& v)
    {
        return Point3<T>(-v.x, -v.y, -v.z);
    }
}

#include "Vec3Float.h"
