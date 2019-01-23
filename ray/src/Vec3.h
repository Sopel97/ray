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
        return
            lhs.x * rhs.x +
            lhs.y * rhs.y +
            lhs.z * rhs.z;
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
