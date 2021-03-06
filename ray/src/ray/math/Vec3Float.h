#pragma once

#include "Vec3.h"

#include "m128/M128Math.h"

#include <xmmintrin.h>
#include <smmintrin.h>

#include "m128/M128MemberSwizzleGeneratorDef.h"

namespace ray
{
    template <>
    struct Vec3<float>;

    template <>
    struct Vec3Mask<float>
    {
        __m128 xmm;

        explicit Vec3Mask(__m128 xmm) noexcept :
            xmm(xmm)
        {
        }

        [[nodiscard]] std::uint8_t packed() const
        {
            return _mm_movemask_ps(xmm);
        }
    };

    template <>
    struct alignas(alignof(__m128)) UnitVec3<float>
    {
        union {
            struct {
                float x, y, z, _;
            };
            __m128 xmm;
        };

        RAY_GEN_MEMBER_SWIZZLE3_ALL(UnitVec3<float>)

        [[nodiscard]] static UnitVec3<float> xAxis() noexcept
        {
            return UnitVec3<float>(AssumeNormalized{}, 1, 0, 0);
        }

        [[nodiscard]] static UnitVec3<float> yAxis() noexcept
        {
            return UnitVec3<float>(AssumeNormalized{}, 0, 1, 0);
        }

        [[nodiscard]] static UnitVec3<float> zAxis() noexcept
        {
            return UnitVec3<float>(AssumeNormalized{}, 0, 0, 1);
        }

        UnitVec3() noexcept :
            xmm(_mm_set_ps(0.0f, 0.0f, 0.0f, 1.0f))
        {
        }

        UnitVec3(AssumeNormalized, __m128 xmm) noexcept :
            xmm(xmm)
        {
        }

        UnitVec3(float x, float y, float z) noexcept;
        UnitVec3(AssumeNormalized, const Vec3<float>& vec) noexcept;

        UnitVec3(AssumeNormalized, float x, float y, float z) noexcept :
            xmm(_mm_set_ps(0.0f, z, y, x))
        {
        }

        UnitVec3(const UnitVec3<float>&) noexcept = default;
        UnitVec3(UnitVec3<float>&&) noexcept = default;

        UnitVec3<float>& operator=(const UnitVec3<float>&) noexcept = default;
        UnitVec3<float>& operator=(UnitVec3<float>&&) noexcept = default;

        [[nodiscard]] operator Vec3<float>() const;

        void negate(const Vec3Mask<float>& mask)
        {
            xmm = m128::neg(xmm, mask.xmm);
        }

    protected:
        explicit UnitVec3(__m128 xmm) noexcept :
            xmm(xmm)
        {
        }
    };
    static_assert(sizeof(UnitVec3f) == sizeof(__m128));

    static_assert(sizeof(Normal3f) == sizeof(__m128));

    template <>
    struct alignas(alignof(__m128)) Vec3<float>
    {
        union {
            struct {
                float x, y, z, _;
            };
            __m128 xmm;
        };

        RAY_GEN_MEMBER_SWIZZLE3_ALL(Vec3<float>)

        [[nodiscard]] static Vec3<float> broadcast(float xyz) noexcept
        {
            return Vec3<float>(_mm_set1_ps(xyz));
        }
        [[nodiscard]] static Vec3<float> blend(float a, float b, const Vec3Mask<float>& mask) noexcept
        {
            return Vec3<float>(m128::blend(a, b, mask.xmm));
        }
        [[nodiscard]] static Vec3<float> blend(const Vec3<float>& a, const Vec3<float>& b, const Vec3Mask<float>& mask) noexcept
        {
            return Vec3<float>(m128::blend(a.xmm, b.xmm, mask.xmm));
        }

        Vec3() noexcept :
            xmm(_mm_setzero_ps())
        {
        }

        Vec3(float v) :
            xmm(_mm_set1_ps(v))
        {
        }

        explicit Vec3(__m128 xyz_) noexcept :
            xmm(xyz_)
        {
        }
        Vec3(float x, float y, float z) noexcept :
            xmm(_mm_set_ps(0.0f, z, y, x))
        {

        }
        Vec3(const Vec3<float>&) noexcept = default;
        Vec3(Vec3<float>&&) noexcept = default;

        Vec3<float>& operator=(const Vec3<float>&) noexcept = default;
        Vec3<float>& operator=(Vec3<float>&&) noexcept = default;

        Vec3<float>& operator+=(const Vec3<float>& rhs)
        {
            xmm = m128::add(xmm, rhs.xmm);
            return *this;
        }

        Vec3<float>& operator-=(const Vec3<float>& rhs)
        {
            xmm = m128::sub(xmm, rhs.xmm);
            return *this;
        }

        Vec3<float>& operator*=(const Vec3<float>& rhs)
        {
            xmm = m128::mul(xmm, rhs.xmm);
            return *this;
        }

        Vec3<float>& operator/=(const Vec3<float>& rhs)
        {
            xmm = m128::div(xmm, rhs.xmm);
            return *this;
        }

        [[nodiscard]] float length() const
        {
            using std::sqrt;
            return sqrt(lengthSqr());
        }

        [[nodiscard]] float lengthSqr() const
        {
            return m128::dot3(xmm, xmm);
        }

        [[nodiscard]] float invLength() const
        {
            return static_cast<float>(1) / length();
        }

        void normalize()
        {
            *this *= invLength();
        }

        [[nodiscard]] UnitVec3<float> assumeNormalized() const
        {
            return UnitVec3<float>(AssumeNormalized{}, *this);
        }

        [[nodiscard]] UnitVec3<float> normalized() const
        {
            return UnitVec3<float>(AssumeNormalized{}, m128::mul(xmm, invLength()));
        }

        [[nodiscard]] float max() const
        {
            using std::max;
            return max(max(x, y), z);
        }

        [[nodiscard]] float min() const
        {
            using std::min;
            return min(min(x, y), z);
        }
    };
    static_assert(sizeof(Vec3f) == sizeof(__m128));

    template <>
    struct alignas(alignof(__m128)) Point3<float>
    {
        union {
            struct {
                float x, y, z, _;
            };
            __m128 xmm;
        };

        RAY_GEN_MEMBER_SWIZZLE3_ALL(Point3<float>)

        [[nodiscard]] static Point3<float> broadcast(float xyz) noexcept
        {
            return Point3<float>(_mm_set1_ps(xyz));
        }
        [[nodiscard]] static Point3<float> origin() noexcept
        {
            return Point3<float>{};
        }
        [[nodiscard]] static Point3<float> blend(float a, float b, const Vec3Mask<float>& mask) noexcept
        {
            return Point3<float>(m128::blend(a, b, mask.xmm));
        }
        [[nodiscard]] static Point3<float> blend(const Point3<float>& a, const Point3<float>& b, const Vec3Mask<float>& mask) noexcept
        {
            return Point3<float>(m128::blend(a.xmm, b.xmm, mask.xmm));
        }

        explicit Point3(__m128 xyz_) noexcept :
            xmm(xyz_)
        {
        }
        Point3() noexcept :
            xmm(_mm_setzero_ps())
        {
        }

        Point3(float x, float y, float z) noexcept :
            xmm(_mm_set_ps(0.0f, z, y, x))
        {

        }
        Point3(const Point3<float>&) noexcept = default;
        Point3(Point3<float>&&) noexcept = default;

        Point3<float>& operator=(const Point3<float>&) noexcept = default;
        Point3<float>& operator=(Point3<float>&&) noexcept = default;

        Point3<float>& operator+=(const Vec3<float>& rhs)
        {
            xmm = m128::add(xmm, rhs.xmm);
            return *this;
        }

        Point3<float>& operator-=(const Vec3<float>& rhs)
        {
            xmm = m128::sub(xmm, rhs.xmm);
            return *this;
        }

        [[nodiscard]] Vec3<float> asVector() const
        {
            return Vec3<float>(xmm);
        }

        [[nodiscard]] explicit operator Vec3<float>() const
        {
            return Vec3<float>(xmm);
        }
    };
    static_assert(sizeof(Point3f) == sizeof(__m128));

    inline UnitVec3<float>::UnitVec3(float x, float y, float z) noexcept :
        UnitVec3(Vec3<float>(x, y, z).normalized())
    {
    }

    inline UnitVec3<float>::UnitVec3(AssumeNormalized, const Vec3<float>& vec) noexcept :
        UnitVec3(AssumeNormalized{}, vec.xmm)
    {
    }

    [[nodiscard]] inline UnitVec3<float>::operator Vec3<float>() const
    {
        return Vec3<float>(xmm);
    }

    // Operations

    [[nodiscard]] inline Vec3Mask<float> operator|(const Vec3Mask<float>& lhs, const Vec3Mask<float>& rhs)
    {
        return Vec3Mask<float>(_mm_or_ps(lhs.xmm, rhs.xmm));
    }

    [[nodiscard]] inline Vec3Mask<float> operator&(const Vec3Mask<float>& lhs, const Vec3Mask<float>& rhs)
    {
        return Vec3Mask<float>(_mm_and_ps(lhs.xmm, rhs.xmm));
    }

    [[nodiscard]] inline Vec3Mask<float> operator^(const Vec3Mask<float>& lhs, const Vec3Mask<float>& rhs)
    {
        return Vec3Mask<float>(_mm_xor_ps(lhs.xmm, rhs.xmm));
    }

    [[nodiscard]] inline Vec3Mask<float> operator~(const Vec3Mask<float>& lhs)
    {
        return Vec3Mask<float>(_mm_xor_ps(lhs.xmm, _mm_castsi128_ps(_mm_set1_epi32(0xFFFFFFFF))));
    }

    [[nodiscard]] inline Vec3<float> operator-(const Point3<float>& lhs, const Point3<float>& rhs)
    {
        return Vec3<float>(m128::sub(lhs.xmm, rhs.xmm));
    }

    [[nodiscard]] inline Point3<float> operator-(const Point3<float>& lhs, const Vec3<float>& rhs)
    {
        return Point3<float>(m128::sub(lhs.xmm, rhs.xmm));
    }

    [[nodiscard]] inline Vec3<float> operator-(const Vec3<float>& lhs, const Vec3<float>& rhs)
    {
        return Vec3<float>(m128::sub(lhs.xmm, rhs.xmm));
    }


    [[nodiscard]] inline Point3<float> operator+(const Point3<float>& lhs, const Vec3<float>& rhs)
    {
        return Point3<float>(m128::add(lhs.xmm, rhs.xmm));
    }

    [[nodiscard]] inline Vec3<float> operator+(const Vec3<float>& lhs, const Vec3<float>& rhs)
    {
        return Vec3<float>(m128::add(lhs.xmm, rhs.xmm));
    }


    [[nodiscard]] inline Vec3<float> operator*(const Vec3<float>& lhs, const Vec3<float>& rhs)
    {
        return Vec3<float>(m128::mul(lhs.xmm, rhs.xmm));
    }

    [[nodiscard]] inline Point3<float> operator*(const Point3<float>& lhs, const Vec3<float>& rhs)
    {
        return Point3<float>(m128::mul(lhs.xmm, rhs.xmm));
    }

    [[nodiscard]] inline Vec3<float> operator/(const Vec3<float>& lhs, const Vec3<float>& rhs)
    {
        return Vec3<float>(m128::div(lhs.xmm, rhs.xmm));
    }

    [[nodiscard]] inline Point3<float> operator/(const Point3<float>& lhs, const Vec3<float>& rhs)
    {
        return Point3<float>(m128::div(lhs.xmm, rhs.xmm));
    }


    [[nodiscard]] inline Vec3Mask<float> operator==(const Vec3<float> & lhs, const Vec3<float>& rhs) noexcept
    {
        return Vec3Mask<float>{m128::cmpeq(lhs.xmm, rhs.xmm)};
    }

    [[nodiscard]] inline Vec3Mask<float> operator<(const Vec3<float> & lhs, const Vec3<float>& rhs) noexcept
    {
        return Vec3Mask<float>{m128::cmplt(lhs.xmm, rhs.xmm)};
    }

    [[nodiscard]] inline Vec3Mask<float> operator>(const Vec3<float> & lhs, const Vec3<float>& rhs) noexcept
    {
        return Vec3Mask<float>{m128::cmplt(rhs.xmm, lhs.xmm)};
    }

    [[nodiscard]] inline Vec3Mask<float> operator<=(const Vec3<float> & lhs, const Vec3<float>& rhs) noexcept
    {
        return Vec3Mask<float>{m128::cmple(lhs.xmm, rhs.xmm)};
    }
    [[nodiscard]] inline Vec3Mask<float> operator==(const Point3<float> & lhs, const Point3<float>& rhs) noexcept
    {
        return Vec3Mask<float>{m128::cmpeq(lhs.xmm, rhs.xmm)};
    }

    [[nodiscard]] inline Vec3Mask<float> operator<(const Point3<float> & lhs, const Point3<float>& rhs) noexcept
    {
        return Vec3Mask<float>{m128::cmplt(lhs.xmm, rhs.xmm)};
    }

    [[nodiscard]] inline Vec3Mask<float> operator>(const Point3<float> & lhs, const Point3<float>& rhs) noexcept
    {
        return Vec3Mask<float>{m128::cmplt(rhs.xmm, lhs.xmm)};
    }

    [[nodiscard]] inline Vec3Mask<float> operator<=(const Point3<float> & lhs, const Point3<float>& rhs) noexcept
    {
        return Vec3Mask<float>{m128::cmple(lhs.xmm, rhs.xmm)};
    }

    [[nodiscard]] inline Point3<float> mod(const Point3<float>& lhs, const Vec3<float>& rhs)
    {
        return Point3<float>(m128::mod(lhs.xmm, rhs.xmm));
    }

    [[nodiscard]] inline Vec3<float> rcp(const Vec3<float>& v)
    {
        return 1.0f / v;
    }

    [[nodiscard]] inline Vec3<float> abs(const Vec3<float>& v)
    {
        return Vec3<float>(m128::abs(v.xmm));
    }

    [[nodiscard]] inline UnitVec3<float> abs(const UnitVec3<float>& v)
    {
        return UnitVec3<float>(AssumeNormalized{}, m128::abs(v.xmm));
    }

    [[nodiscard]] inline Point3<float> abs(const Point3<float>& v)
    {
        return Point3<float>(m128::abs(v.xmm));
    }


    [[nodiscard]] inline float dot(const Vec3<float>& lhs, const Vec3<float>& rhs)
    {
        return m128::dot3(lhs.xmm, rhs.xmm);
    }

    [[nodiscard]] inline Vec3<float> cross(const Vec3<float>& lhs, const Vec3<float>& rhs)
    {
        return Vec3<float>(m128::cross3(lhs.xmm, rhs.xmm));
    }

    [[nodiscard]] inline Vec3<float> min(const Vec3<float>& lhs, const Vec3<float>& rhs)
    {
        return Vec3<float>(m128::min(lhs.xmm, rhs.xmm));
    }

    [[nodiscard]] inline Point3<float> min(const Point3<float>& lhs, const Point3<float>& rhs)
    {
        return Point3<float>(m128::min(lhs.xmm, rhs.xmm));
    }


    [[nodiscard]] inline Point3<float> max(const Point3<float>& lhs, const Point3<float>& rhs)
    {
        return Point3<float>(m128::max(lhs.xmm, rhs.xmm));
    }

    [[nodiscard]] inline Vec3<float> max(const Vec3<float>& lhs, const Vec3<float>& rhs)
    {
        return Vec3<float>(m128::max(lhs.xmm, rhs.xmm));
    }


    [[nodiscard]] inline Vec3<float> sqrt(const Vec3<float>& lhs)
    {
        return Vec3<float>(m128::sqrt(lhs.xmm));
    }


    [[nodiscard]] inline UnitVec3<float> operator-(const UnitVec3<float>& vec)
    {
        return UnitVec3<float>(AssumeNormalized{}, m128::neg(vec.xmm));
    }

    [[nodiscard]] inline Normal3<float> operator-(const Normal3<float>& vec)
    {
        return Normal3<float>(AssumeNormalized{}, m128::neg(vec.xmm));
    }

    [[nodiscard]] inline Vec3<float> operator-(const Vec3<float>& vec)
    {
        return Vec3<float>(m128::neg(vec.xmm));
    }

    [[nodiscard]] inline Point3<float> operator-(const Point3<float>& vec)
    {
        return Point3<float>(m128::neg(vec.xmm));
    }
}

#include "m128/M128MemberSwizzleGeneratorUndef.h"
