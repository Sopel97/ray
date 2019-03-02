#pragma once

#include "m128/M128Math.h"

#include <cstdint>
#include <cmath>

#include "m128/M128MemberSwizzleGeneratorDef.h"

namespace ray
{
    struct Float4Mask
    {
        __m128 xmm;

        explicit Float4Mask(__m128 xmm) :
            xmm(xmm)
        {
        }

        std::uint8_t packed() const
        {
            return _mm_movemask_ps(xmm);
        }
    };

    struct alignas(alignof(__m128)) Float4
    {
        union {
            struct {
                float v[4];
            };
            __m128 xmm;
        };

        RAY_GEN_MEMBER_SWIZZLE4_DIG_ALL(Float4)

        static Float4 broadcast(float xyz)
        {
            return Float4(_mm_set1_ps(xyz));
        }
        static Float4 blend(float a, float b, const Float4Mask& mask)
        {
            return Float4(m128::blend(a, b, mask.xmm));
        }
        static Float4 blend(const Float4& a, const Float4& b, const Float4Mask& mask)
        {
            return Float4(m128::blend(a.xmm, b.xmm, mask.xmm));
        }

        Float4() :
            xmm(_mm_setzero_ps())
        {
        }

        explicit Float4(__m128 xmm) :
            xmm(xmm)
        {
        }
        Float4(float v0, float v1, float v2, float v3) :
            xmm(_mm_set_ps(v3, v2, v1, v0))
        {

        }
        Float4(const Float4&) = default;
        Float4(Float4&&) noexcept = default;

        Float4& operator=(const Float4&) = default;
        Float4& operator=(Float4&&) noexcept = default;

        Float4& operator+=(const Float4& rhs)
        {
            xmm = m128::add(xmm, rhs.xmm);
            return *this;
        }

        Float4& operator-=(const Float4& rhs)
        {
            xmm = m128::sub(xmm, rhs.xmm);
            return *this;
        }

        Float4& operator*=(const Float4& rhs)
        {
            xmm = m128::mul(xmm, rhs.xmm);
            return *this;
        }

        Float4& operator/=(const Float4& rhs)
        {
            xmm = m128::div(xmm, rhs.xmm);
            return *this;
        }

        Float4& operator*=(float rhs)
        {
            xmm = m128::mul(xmm, rhs);
            return *this;
        }

        Float4& operator/=(float rhs)
        {
            xmm = m128::div(xmm, rhs);
            return *this;
        }

        float max() const
        {
            return m128::hmax(xmm);
        }

        float min() const
        {
            return m128::hmin(xmm);
        }
    };
    static_assert(sizeof(Float4) == sizeof(__m128));

    inline Float4Mask operator|(const Float4Mask& lhs, const Float4Mask& rhs)
    {
        return Float4Mask(_mm_or_ps(lhs.xmm, rhs.xmm));
    }

    inline Float4Mask operator&(const Float4Mask& lhs, const Float4Mask& rhs)
    {
        return Float4Mask(_mm_and_ps(lhs.xmm, rhs.xmm));
    }

    inline Float4Mask operator^(const Float4Mask& lhs, const Float4Mask& rhs)
    {
        return Float4Mask(_mm_xor_ps(lhs.xmm, rhs.xmm));
    }

    inline Float4 operator-(const Float4& f)
    {
        return Float4(m128::neg(f.xmm));
    }

    inline Float4 operator+(const Float4& lhs, const Float4& rhs)
    {
        return Float4(m128::add(lhs.xmm, rhs.xmm));
    }

    inline Float4 operator-(const Float4& lhs, const Float4& rhs)
    {
        return Float4(m128::sub(lhs.xmm, rhs.xmm));
    }

    inline Float4 operator*(const Float4& lhs, const Float4& rhs)
    {
        return Float4(m128::mul(lhs.xmm, rhs.xmm));
    }

    inline Float4 operator/(const Float4& lhs, const Float4& rhs)
    {
        return Float4(m128::div(lhs.xmm, rhs.xmm));
    }

    inline Float4 operator*(const Float4& lhs, float rhs)
    {
        return Float4(m128::mul(lhs.xmm, rhs));
    }

    inline Float4 operator/(const Float4& lhs, float rhs)
    {
        return Float4(m128::div(lhs.xmm, rhs));
    }

    inline Float4 operator*(float lhs, const Float4& rhs)
    {
        return Float4(m128::mul(lhs, rhs.xmm));
    }

    inline Float4 operator/(float lhs, const Float4& rhs)
    {
        return Float4(m128::div(lhs, rhs.xmm));
    }

    inline Float4Mask operator==(const Float4& lhs, const Float4& rhs)
    {
        return Float4Mask(m128::cmpeq(lhs.xmm, rhs.xmm));
    }

    inline Float4Mask operator==(const Float4& lhs, float rhs)
    {
        return Float4Mask(m128::cmpeq(lhs.xmm, rhs));
    }

    inline Float4Mask operator<(const Float4& lhs, const Float4& rhs)
    {
        return Float4Mask(m128::cmplt(lhs.xmm, rhs.xmm));
    }

    inline Float4Mask operator<(const Float4& lhs, float rhs)
    {
        return Float4Mask(m128::cmplt(lhs.xmm, rhs));
    }

    inline Float4Mask operator>(const Float4& lhs, const Float4& rhs)
    {
        return Float4Mask(m128::cmplt(rhs.xmm, lhs.xmm));
    }

    inline Float4Mask operator>(const Float4& lhs, float rhs)
    {
        return Float4Mask(m128::cmplt(rhs, lhs.xmm));
    }

    inline Float4Mask operator<=(const Float4& lhs, const Float4& rhs)
    {
        return Float4Mask(m128::cmpeq(lhs.xmm, rhs.xmm));
    }

    inline Float4Mask operator<=(const Float4& lhs, float rhs)
    {
        return Float4Mask(m128::cmpeq(lhs.xmm, rhs));
    }

    inline Float4Mask operator>=(const Float4& lhs, const Float4& rhs)
    {
        return Float4Mask(m128::cmpeq(rhs.xmm, lhs.xmm));
    }

    inline Float4Mask operator>=(const Float4& lhs, float rhs)
    {
        return Float4Mask(m128::cmpeq(rhs, lhs.xmm));
    }

    inline Float4 sqrt(const Float4& f)
    {
        return Float4(m128::sqrt(f.xmm));
    }

    inline Float4 max(const Float4& a, const Float4& b)
    {
        return Float4(m128::max(a.xmm, b.xmm));
    }

    inline Float4 min(const Float4& a, const Float4& b)
    {
        return Float4(m128::min(a.xmm, b.xmm));
    }

    inline Float4 abs(const Float4& a)
    {
        return Float4(m128::abs(a.xmm));
    }
}

#include "m128/M128MemberSwizzleGeneratorUndef.h"
