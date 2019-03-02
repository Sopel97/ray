#pragma once

#include "Float4.h"
#include "Vec3.h"

namespace ray
{
    template <typename T>
    struct Vec3x4;

    template <>
    struct alignas(alignof(__m128)) Vec3x4<float>
    {
        Float4 x;
        Float4 y;
        Float4 z;

        static Vec3x4<float> broadcast(const Float4& xyz)
        {
            return Vec3x4<float>(xyz, xyz, xyz);
        }

        static Vec3x4<float> broadcast(const Vec3<float>& v)
        {
            return Vec3x4<float>(
                _mm_set1_ps(v.x),
                _mm_set1_ps(v.y),
                _mm_set1_ps(v.z)
            );
        }

        Vec3x4() = default;

        Vec3x4(__m128 x, __m128 y, __m128 z) :
            x(x),
            y(y),
            z(z)
        {
        }

        Vec3x4(const Float4& x, const Float4& y, const Float4& z) :
            x(x),
            y(y),
            z(z)
        {
        }
        Vec3x4(const Vec3x4<float>&) = default;
        Vec3x4(Vec3x4<float>&&) noexcept = default;

        Vec3x4<float>& operator=(const Vec3x4<float>&) = default;
        Vec3x4<float>& operator=(Vec3x4<float>&&) noexcept = default;

        Vec3x4<float>& operator+=(const Vec3x4<float>& rhs)
        {
            x += rhs.x;
            y += rhs.y;
            z += rhs.z;
            return *this;
        }

        Vec3x4<float>& operator-=(const Vec3x4<float>& rhs)
        {
            x -= rhs.x;
            y -= rhs.y;
            z -= rhs.z;
            return *this;
        }

        Vec3x4<float>& operator*=(const Vec3x4<float>& rhs)
        {
            x *= rhs.x;
            y *= rhs.y;
            z *= rhs.z;
            return *this;
        }

        Vec3x4<float>& operator/=(const Vec3x4<float>& rhs)
        {
            x /= rhs.x;
            y /= rhs.y;
            z /= rhs.z;
            return *this;
        }

        Vec3x4<float>& operator*=(float rhs)
        {
            x *= rhs;
            y *= rhs;
            z *= rhs;
            return *this;
        }

        Vec3x4<float>& operator*=(const Float4& rhs)
        {
            x *= rhs;
            y *= rhs;
            z *= rhs;
            return *this;
        }

        Vec3x4<float>& operator/=(float rhs)
        {
            x /= rhs;
            y /= rhs;
            z /= rhs;
            return *this;
        }

        Vec3x4<float>& operator/=(const Float4& rhs)
        {
            x /= rhs;
            y /= rhs;
            z /= rhs;
            return *this;
        }

        Float4 length() const
        {
            return sqrt(lengthSqr());
        }

        Float4 lengthSqr() const
        {
            return x*x + y*y + z*z;
        }

        Float4 invLength() const
        {
            return 1.0f / length();
        }

        void normalize()
        {
            *this *= invLength();
        }

        Vec3x4<float> normalized() const
        {
            Vec3x4<float> c(*this);
            c.normalize();
            return c;
        }

        Float4 max() const
        {
            return ray::max(ray::max(x, y), z);
        }

        Float4 min() const
        {
            return ray::min(ray::min(x, y), z);
        }
    };

    inline Vec3x4<float> operator-(const Vec3x4<float>& v)
    {
        return Vec3x4<float>(
            -v.x,
            -v.y,
            -v.z
        );
    }

    inline Vec3x4<float> operator+(const Vec3x4<float>& lhs, const Vec3x4<float>& rhs)
    {
        return Vec3x4<float>(
            lhs.x + rhs.x,
            lhs.y + rhs.y,
            lhs.z + rhs.z
        );
    }

    inline Vec3x4<float> operator-(const Vec3x4<float>& lhs, const Vec3x4<float>& rhs)
    {
        return Vec3x4<float>(
            lhs.x - rhs.x,
            lhs.y - rhs.y,
            lhs.z - rhs.z
            );
    }

    inline Vec3x4<float> operator*(const Vec3x4<float>& lhs, const Vec3x4<float>& rhs)
    {
        return Vec3x4<float>(
            lhs.x * rhs.x,
            lhs.y * rhs.y,
            lhs.z * rhs.z
            );
    }

    inline Vec3x4<float> operator*(const Vec3x4<float>& lhs, const Float4& rhs)
    {
        return Vec3x4<float>(
            lhs.x * rhs,
            lhs.y * rhs,
            lhs.z * rhs
            );
    }

    inline Vec3x4<float> operator/(const Vec3x4<float>& lhs, const Vec3x4<float>& rhs)
    {
        return Vec3x4<float>(
            lhs.x / rhs.x,
            lhs.y / rhs.y,
            lhs.z / rhs.z
        );
    }

    inline Vec3x4<float> operator/(const Vec3x4<float>& lhs, const Float4& rhs)
    {
        return Vec3x4<float>(
            lhs.x / rhs,
            lhs.y / rhs,
            lhs.z / rhs
        );
    }

    inline Vec3x4<float> abs(const Vec3x4<float>& v)
    {
        return Vec3x4<float>(
            abs(v.x),
            abs(v.y),
            abs(v.z)
        );
    }

    inline Float4 dot(const Vec3x4<float>& lhs, const Vec3x4<float>& rhs)
    {
        return lhs.x*rhs.x + lhs.y*rhs.y + lhs.z*rhs.z;
    }

    inline Vec3x4<float> cross(const Vec3x4<float>& lhs, const Vec3x4<float>& rhs)
    {
        return Vec3x4<float>(
            lhs.y*rhs.z - lhs.z*rhs.y,
            lhs.z*rhs.x - lhs.x*rhs.z,
            lhs.x*rhs.y - lhs.y*rhs.x
        );
    }
}
