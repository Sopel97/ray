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

        [[nodiscard]] static Vec3x4<float> broadcast(const Float4& xyz) noexcept
        {
            return Vec3x4<float>(xyz, xyz, xyz);
        }

        [[nodiscard]] static Vec3x4<float> broadcast(const Vec3<float>& v) noexcept
        {
            return Vec3x4<float>(
                _mm_set1_ps(v.x),
                _mm_set1_ps(v.y),
                _mm_set1_ps(v.z)
            );
        }

        Vec3x4() noexcept = default;

        Vec3x4(__m128 x, __m128 y, __m128 z) noexcept :
            x(x),
            y(y),
            z(z)
        {
        }

        Vec3x4(const Float4& x, const Float4& y, const Float4& z) noexcept :
            x(x),
            y(y),
            z(z)
        {
        }

        Vec3x4(const Vec3<float>& v0, const Vec3<float>& v1, const Vec3<float>& v2, const Vec3<float>& v3) noexcept
        {
            __m128 coords[4] = { v0.xmm, v1.xmm, v2.xmm, v3.xmm };
            m128::transpose(coords);
            x = Float4(coords[0]);
            y = Float4(coords[1]);
            z = Float4(coords[2]);
        }

        Vec3x4(const Vec3x4<float>&) noexcept = default;
        Vec3x4(Vec3x4<float>&&) noexcept = default;

        Vec3x4<float>& operator=(const Vec3x4<float>&) noexcept = default;
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

        void insert(const Vec3<float>& v, int i)
        {
            x.insert(v.x, i);
            y.insert(v.y, i);
            z.insert(v.z, i);
        }

        [[nodiscard]] Float4 length() const
        {
            return sqrt(lengthSqr());
        }

        [[nodiscard]] Float4 lengthSqr() const
        {
            return x*x + y*y + z*z;
        }

        [[nodiscard]] Float4 invLength() const
        {
            return 1.0f / length();
        }

        void normalize()
        {
            *this *= invLength();
        }

        [[nodiscard]] Vec3x4<float> normalized() const
        {
            Vec3x4<float> c(*this);
            c.normalize();
            return c;
        }

        [[nodiscard]] Float4 max() const
        {
            return ray::max(ray::max(x, y), z);
        }

        [[nodiscard]] Float4 min() const
        {
            return ray::min(ray::min(x, y), z);
        }
    };

    [[nodiscard]] inline Vec3x4<float> operator-(const Vec3x4<float>& v)
    {
        return Vec3x4<float>(
            -v.x,
            -v.y,
            -v.z
        );
    }

    [[nodiscard]] inline Vec3x4<float> operator+(const Vec3x4<float>& lhs, const Vec3x4<float>& rhs)
    {
        return Vec3x4<float>(
            lhs.x + rhs.x,
            lhs.y + rhs.y,
            lhs.z + rhs.z
        );
    }

    [[nodiscard]] inline Vec3x4<float> operator-(const Vec3x4<float>& lhs, const Vec3x4<float>& rhs)
    {
        return Vec3x4<float>(
            lhs.x - rhs.x,
            lhs.y - rhs.y,
            lhs.z - rhs.z
            );
    }

    [[nodiscard]] inline Vec3x4<float> operator*(const Vec3x4<float>& lhs, const Vec3x4<float>& rhs)
    {
        return Vec3x4<float>(
            lhs.x * rhs.x,
            lhs.y * rhs.y,
            lhs.z * rhs.z
            );
    }

    [[nodiscard]] inline Vec3x4<float> operator*(const Vec3x4<float>& lhs, const Float4& rhs)
    {
        return Vec3x4<float>(
            lhs.x * rhs,
            lhs.y * rhs,
            lhs.z * rhs
            );
    }

    [[nodiscard]] inline Vec3x4<float> operator/(const Vec3x4<float>& lhs, const Vec3x4<float>& rhs)
    {
        return Vec3x4<float>(
            lhs.x / rhs.x,
            lhs.y / rhs.y,
            lhs.z / rhs.z
        );
    }

    [[nodiscard]] inline Vec3x4<float> operator/(const Vec3x4<float>& lhs, const Float4& rhs)
    {
        return Vec3x4<float>(
            lhs.x / rhs,
            lhs.y / rhs,
            lhs.z / rhs
        );
    }

    [[nodiscard]] inline Vec3x4<float> abs(const Vec3x4<float>& v)
    {
        return Vec3x4<float>(
            abs(v.x),
            abs(v.y),
            abs(v.z)
        );
    }

    [[nodiscard]] inline Float4 dot(const Vec3x4<float>& lhs, const Vec3x4<float>& rhs)
    {
        return lhs.x*rhs.x + lhs.y*rhs.y + lhs.z*rhs.z;
    }

    [[nodiscard]] inline Vec3x4<float> cross(const Vec3x4<float>& lhs, const Vec3x4<float>& rhs)
    {
        return Vec3x4<float>(
            lhs.y*rhs.z - lhs.z*rhs.y,
            lhs.z*rhs.x - lhs.x*rhs.z,
            lhs.x*rhs.y - lhs.y*rhs.x
        );
    }
}
