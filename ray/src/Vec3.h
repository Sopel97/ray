#pragma once

namespace ray
{
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

        constexpr void normalize()
        {
            *this *= invLength();
        }

        constexpr Vec3<T> normalized() const
        {
            return *this * invLength();
        }
    };

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
    constexpr T dot(const Vec3<T>& lhs, const Vec3<T>& rhs)
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
    constexpr T mixed(const Vec3<T>& a, const Vec3<T>& b, const Vec3<T>& c)
    {
        return dot(a, cross(b, c));
    }

    // projection of v on n
    template <typename T>
    constexpr T projection(const Vec3<T>& v, const Vec3<T>& n)
    {
        return n * (dot(v, n) / dot(n, n));
    }

    // reflection of v from a surface with normal n (like light reflection)
    template <typename T>
    constexpr Vec3<T> reflection(const Vec3<T>& v, const Vec3<T>& n)
    {
        return v - static_cast<T>(2) * projection(v, n);
    }

    // refraction of v through a surface with normal n with refractive index ratio of r
    template <typename T>
    constexpr Vec3<T> refraction(const Vec3<T>& v, const Vec3<T>& n, const T& r)
    {
        using std::sqrt;

        const T c = dot(v, n);
        const T s = sqrt(static_cast<T>(1) - r*r*(static_cast<T>(1) - c*c));
        return r*v + (r*c - s)*n;
    }

    template <typename T>
    constexpr T distance(const Vec3<T>& lhs, const Vec3<T>& rhs)
    {
        return (rhs - lhs).length();
    }

    template <typename T>
    constexpr T distanceSqr(const Vec3<T>& lhs, const Vec3<T>& rhs)
    {
        return (rhs - lhs).lengthSqr();
    }

    template <typename T>
    constexpr Vec3<T> operator-(const Vec3<T>& v)
    {
        return Vec3<T>(-v.x, -v.y, -v.z);
    }

    using Vec3f = Vec3<float>;
}
