#pragma once

#include <cmath>

namespace ray
{
    template <typename T>
    struct Vec3;

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

    template <typename T>
    struct Normal3
    {
        detail::OwnedReadOnlyProperty<T, Normal3<T>> x, y, z;

        constexpr Normal3() :
            x(1),
            y(0),
            z(0)
        {
        }

        constexpr Normal3(const T& x, const T& y, const T& z) :
            Normal3(Vec3<T>(x, y, z).normalized())
        {
        }

        constexpr explicit Normal3(const Vec3<T>& v) :
            Normal3(v.normalized())
        {
        }

        constexpr Normal3(AssumeNormalized, const T& x, const T& y, const T& z) :
            x(x),
            y(y),
            z(z)
        {
        }

        constexpr Normal3(AssumeNormalized, const Vec3<T>& v) :
            x(v.x),
            y(v.y),
            z(v.z)
        {
        }

        constexpr Normal3(const Normal3<T>&) = default;
        constexpr Normal3(Normal3<T>&&) noexcept = default;

        constexpr Normal3<T>& operator=(const Normal3<T>&) = default;
        constexpr Normal3<T>& operator=(Normal3<T>&&) noexcept = default;

        constexpr operator Vec3<T>() const
        {
            return Vec3<T>(x, y, z);
        }

        constexpr Vec3<T> inv() const
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

        constexpr Vec3() = default;
        constexpr Vec3(const T& x, const T& y, const T& z) :
            x(x),
            y(y),
            z(z)
        {

        }
        constexpr Vec3(const Vec3<T>&) = default;
        constexpr Vec3(Vec3<T>&&) noexcept = default;

        constexpr Vec3<T>& operator=(const Vec3<T>&) = default;
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

        constexpr Vec3<T>& operator*=(const T& rhs)
        {
            x *= rhs;
            y *= rhs;
            z *= rhs;
            return *this;
        }

        constexpr Vec3<T>& operator/=(const T& rhs)
        {
            x /= rhs;
            y /= rhs;
            z /= rhs;
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
                y * y +
                z * z;
        }

        constexpr T invLength() const
        {
            return static_cast<T>(1) / length();
        }

        constexpr Vec3<T> inv() const
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

        constexpr Normal3<T> assumeNormalized() const
        {
            return Normal3<T>(AssumeNormalized{}, *this);
        }

        constexpr Normal3<T> normalized() const
        {
            return Normal3<T>(AssumeNormalized{}, *this * invLength());
        }
    };

    using Vec3f = Vec3<float>;

    template <typename T>
    struct Point3
    {
        T x, y, z;

        constexpr static Point3<T> origin()
        {
            return Point3<T>(0, 0, 0);
        }

        constexpr Point3() = default;
        constexpr Point3(const T& x, const T& y, const T& z) :
            x(x),
            y(y),
            z(z)
        {

        }
        constexpr Point3(const Point3<T>&) = default;
        constexpr Point3(Point3<T>&&) noexcept = default;

        constexpr Point3<T>& operator=(const Point3<T>&) = default;
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
    };

    using Point3f = Point3<float>;

    // Operations

    template <typename T>
    constexpr Vec3<T> operator-(const Point3<T>& lhs, const Point3<T>& rhs)
    {
        return Vec3<T>(
            lhs.x - rhs.x,
            lhs.y - rhs.y,
            lhs.z - rhs.z
            );
    }

    template <typename T>
    constexpr Point3<T> operator-(const Point3<T>& lhs, const Vec3<T>& rhs)
    {
        return Point3<T>(
            lhs.x - rhs.x,
            lhs.y - rhs.y,
            lhs.z - rhs.z
            );
    }

    template <typename T>
    constexpr Point3<T> operator+(const Point3<T>& lhs, const Vec3<T>& rhs)
    {
        return Point3<T>(
            lhs.x + rhs.x,
            lhs.y + rhs.y,
            lhs.z + rhs.z
            );
    }

    template <typename T>
    constexpr Vec3<T> operator+(const Vec3<T>& lhs, const Vec3<T>& rhs)
    {
        return Vec3<T>(
            lhs.x + rhs.x,
            lhs.y + rhs.y,
            lhs.z + rhs.z
        );
    }

    template <typename T>
    constexpr Vec3<T> operator-(const Vec3<T>& lhs, const Vec3<T>& rhs)
    {
        return Vec3<T>(
            lhs.x - rhs.x,
            lhs.y - rhs.y,
            lhs.z - rhs.z
            );
    }

    template <typename T>
    constexpr Vec3<T> operator*(const Vec3<T>& lhs, const Vec3<T>& rhs)
    {
        return Vec3<T>(
            lhs.x * rhs.x,
            lhs.y * rhs.y,
            lhs.z * rhs.z
            );
    }

    template <typename T>
    constexpr Vec3<T> operator/(const Vec3<T>& lhs, const Vec3<T>& rhs)
    {
        return Vec3<T>(
            lhs.x / rhs.x,
            lhs.y / rhs.y,
            lhs.z / rhs.z
            );
    }

    template <typename T>
    constexpr Vec3<T> operator*(const Vec3<T>& lhs, const T& rhs)
    {
        return Vec3<T>(
            lhs.x * rhs,
            lhs.y * rhs,
            lhs.z * rhs
            );
    }

    template <typename T>
    constexpr Vec3<T> operator*(const Normal3<T>& lhs, const T& rhs)
    {
        return Vec3<T>(
            lhs.x * rhs,
            lhs.y * rhs,
            lhs.z * rhs
            );
    }

    template <typename T>
    constexpr Vec3<T> operator*(const T& lhs, const Vec3<T>& rhs)
    {
        return Vec3<T>(
            lhs * rhs.x,
            lhs * rhs.y,
            lhs * rhs.z
            );
    }

    template <typename T>
    constexpr Vec3<T> operator/(const Vec3<T>& lhs, const T& rhs)
    {
        return Vec3<T>(
            lhs.x / rhs,
            lhs.y / rhs,
            lhs.z / rhs
            );
    }

    template <typename T>
    constexpr Vec3<T> operator*(const T& lhs, const Normal3<T>& rhs)
    {
        return Vec3<T>(
            lhs * rhs.x,
            lhs * rhs.y,
            lhs * rhs.z
            );
    }

    template <typename T>
    constexpr Vec3<T> operator/(const Normal3<T>& lhs, const T& rhs)
    {
        return Vec3<T>(
            lhs.x / rhs,
            lhs.y / rhs,
            lhs.z / rhs
            );
    }

    template <typename T>
    constexpr T dot(const Vec3<T>& lhs, const Vec3<T>& rhs)
    {
        return
            lhs.x * rhs.x +
            lhs.y * rhs.y +
            lhs.z * rhs.z;
    }

    template <typename T>
    constexpr T dot(const Normal3<T>& lhs, const Vec3<T>& rhs)
    {
        return
            lhs.x * rhs.x +
            lhs.y * rhs.y +
            lhs.z * rhs.z;
    }

    template <typename T>
    constexpr T dot(const Vec3<T>& lhs, const Normal3<T>& rhs)
    {
        return
            lhs.x * rhs.x +
            lhs.y * rhs.y +
            lhs.z * rhs.z;
    }

    template <typename T>
    constexpr T dot(const Normal3<T>& lhs, const Normal3<T>& rhs)
    {
        return
            lhs.x * rhs.x +
            lhs.y * rhs.y +
            lhs.z * rhs.z;
    }

    template <typename T>
    constexpr Vec3<T> cross(const Vec3<T>& lhs, const Vec3<T>& rhs)
    {
        return Vec3<T>(
            lhs.y*rhs.z - lhs.z*rhs.y, 
            lhs.z*rhs.x - lhs.x*rhs.z, 
            lhs.x*rhs.y - lhs.y*rhs.x
        );
    }

    template <typename T>
    constexpr Vec3<T> cross(const Normal3<T>& lhs, const Vec3<T>& rhs)
    {
        return Vec3<T>(
            lhs.y*rhs.z - lhs.z*rhs.y,
            lhs.z*rhs.x - lhs.x*rhs.z,
            lhs.x*rhs.y - lhs.y*rhs.x
            );
    }

    template <typename T>
    constexpr Vec3<T> cross(const Vec3<T>& lhs, const Normal3<T>& rhs)
    {
        return Vec3<T>(
            lhs.y*rhs.z - lhs.z*rhs.y,
            lhs.z*rhs.x - lhs.x*rhs.z,
            lhs.x*rhs.y - lhs.y*rhs.x
            );
    }

    template <typename T>
    constexpr Vec3<T> cross(const Normal3<T>& lhs, const Normal3<T>& rhs)
    {
        return Vec3<T>(
            lhs.y*rhs.z - lhs.z*rhs.y,
            lhs.z*rhs.x - lhs.x*rhs.z,
            lhs.x*rhs.y - lhs.y*rhs.x
            );
    }

    template <typename T>
    constexpr T mixed(const Vec3<T>& a, const Vec3<T>& b, const Vec3<T>& c)
    {
        return dot(a, cross(b, c));
    }

    template <typename T>
    constexpr T distance(const Point3<T>& lhs, const Point3<T>& rhs)
    {
        return (rhs - lhs).length();
    }

    template <typename T>
    constexpr T distanceSqr(const Point3<T>& lhs, const Point3<T>& rhs)
    {
        return (rhs - lhs).lengthSqr();
    }

    // projection of v on n
    template <typename T>
    constexpr Vec3<T> projection(const Vec3<T>& v, const Vec3<T>& n)
    {
        return n * (dot(v, n) / dot(n, n));
    }

    // projection of v on n
    template <typename T>
    constexpr Vec3<T> projection(const Vec3<T>& v, const Normal3<T>& n)
    {
        return n * dot(v, n);
    }

    // projection of v on n
    template <typename T>
    constexpr Vec3<T> projection(const Normal3<T>& v, const Vec3<T>& n)
    {
        return n * (dot(v, n) / dot(n, n));
    }

    // projection of v on n
    template <typename T>
    constexpr Vec3<T> projection(const Normal3<T>& v, const Normal3<T>& n)
    {
        return n * dot(v, n);
    }

    // reflection of v from a surface with normal n (like light reflection)
    template <typename T>
    constexpr Vec3<T> reflection(const Vec3<T>& v, const Vec3<T>& n)
    {
        return v - static_cast<T>(2) * projection(v, n);
    }

    // reflection of v from a surface with normal n (like light reflection)
    template <typename T>
    constexpr Vec3<T> reflection(const Vec3<T>& v, const Normal3<T>& n)
    {
        return v - static_cast<T>(2) * projection(v, n);
    }

    // reflection of v from a surface with normal n (like light reflection)
    template <typename T>
    constexpr Normal3<T> reflection(const Normal3<T>& v, const Vec3<T>& n)
    {
        return (Vec3<T>(v) - static_cast<T>(2) * projection(v, n)).assumeNormalized();
    }

    // reflection of v from a surface with normal n (like light reflection)
    template <typename T>
    constexpr Normal3<T> reflection(const Normal3<T>& v, const Normal3<T>& n)
    {
        return (Vec3<T>(v) - static_cast<T>(2) * projection(v, n)).assumeNormalized();
    }

    // refraction of v through a surface with normal n with refractive index ratio of r
    template <typename T>
    constexpr Vec3<T> refraction(const Vec3<T>& v, const Vec3<T>& n, const T& r)
    {
        using std::sqrt;

        const T c = -dot(v, n);
        const T s = sqrt(static_cast<T>(1) - r * r*(static_cast<T>(1) - c * c));
        return r * v + (r*c - s)*n;
    }

    // refraction of v through a surface with normal n with refractive index ratio of r
    template <typename T>
    constexpr Normal3<T> refraction(const Normal3<T>& v, const Vec3<T>& n, const T& r)
    {
        using std::sqrt;

        const T c = -dot(v, n);
        const T s = sqrt(static_cast<T>(1) - r * r*(static_cast<T>(1) - c * c));
        return (r * v + (r*c - s)*n).normalized();
    }

    // refraction of v through a surface with normal n with refractive index ratio of r
    template <typename T>
    constexpr Vec3<T> refraction(const Vec3<T>& v, const Normal3<T>& n, const T& r)
    {
        using std::sqrt;

        const T c = -dot(v, n);
        const T s = sqrt(static_cast<T>(1) - r * r*(static_cast<T>(1) - c * c));
        return r * v + (r*c - s)*n;
    }

    // refraction of v through a surface with normal n with refractive index ratio of r
    template <typename T>
    constexpr Normal3<T> refraction(const Normal3<T>& v, const Normal3<T>& n, const T& r)
    {
        using std::sqrt;

        const T c = -dot(v, n);
        const T s = sqrt(static_cast<T>(1) - r * r*(static_cast<T>(1) - c * c));
        return (r * v + (r*c - s)*n).assumeNormalized();
    }

    template <typename T>
    constexpr Normal3<T> operator-(const Normal3<T>& v)
    {
        return Normal3<T>(AssumeNormalized{}, -v.x, -v.y, -v.z);
    }

    template <typename T>
    constexpr Vec3<T> operator-(const Vec3<T>& v)
    {
        return Vec3<T>(-v.x, -v.y, -v.z);
    }

    template <typename T>
    constexpr Point3<T> operator-(const Point3<T>& v)
    {
        return Point3<T>(-v.x, -v.y, -v.z);
    }
}
