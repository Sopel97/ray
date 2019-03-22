#pragma once

#include "m128/M128Math.h"
#include "m128/M128ScalarAccessor.h"

#include <cstdint>
#include <cmath>

#include "m128/M128MemberSwizzleGeneratorDef.h"

namespace ray
{
    struct Float4Mask
    {
        __m128 xmm;

        explicit Float4Mask(__m128 xmm) noexcept :
            xmm(xmm)
        {
        }

        [[nodiscard]] std::uint8_t packed() const
        {
            return _mm_movemask_ps(xmm);
        }
    };

    struct alignas(alignof(__m128)) Float4
    {
        union {
            __m128 xmm;
            m128::ScalarAccessor<0> v0;
            m128::ScalarAccessor<1> v1;
            m128::ScalarAccessor<2> v2;
            m128::ScalarAccessor<3> v3;
        };

        RAY_GEN_MEMBER_SWIZZLE4_DIG_ALL(Float4)

        [[nodiscard]] static Float4 broadcast(float xyz) noexcept
        {
            return Float4(_mm_set1_ps(xyz));
        }
        [[nodiscard]] static Float4 blend(float a, float b, const Float4Mask& mask) noexcept
        {
            return Float4(m128::blend(a, b, mask.xmm));
        }
        [[nodiscard]] static Float4 blend(const Float4& a, const Float4& b, const Float4Mask& mask) noexcept
        {
            return Float4(m128::blend(a.xmm, b.xmm, mask.xmm));
        }

        Float4() noexcept :
            xmm(_mm_setzero_ps())
        {
        }

        explicit Float4(__m128 xmm) noexcept :
            xmm(xmm)
        {
        }

        Float4(float v) :
            xmm(_mm_set1_ps(v))
        {
        }

        Float4(float v0, float v1, float v2, float v3) noexcept :
            xmm(_mm_set_ps(v3, v2, v1, v0))
        {

        }

        Float4(const Float4& other) noexcept :
            xmm(other.xmm)
        {

        }
        Float4(Float4&& other) noexcept :
            xmm(other.xmm)
        {

        }

        Float4& operator=(const Float4& other) noexcept
        {
            xmm = other.xmm;
            return *this;
        }
        Float4& operator=(Float4&& other) noexcept
        {
            xmm = other.xmm;
            return *this;
        }

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

        void insert(float x, int i)
        {
            xmm = m128::insert(xmm, x, i);
        }

        [[nodiscard]] float max() const
        {
            return m128::hmax(xmm);
        }

        [[nodiscard]] float min() const
        {
            return m128::hmin(xmm);
        }
    };
    static_assert(sizeof(Float4) == sizeof(__m128));

    [[nodiscard]] inline Float4Mask operator|(const Float4Mask& lhs, const Float4Mask& rhs)
    {
        return Float4Mask(_mm_or_ps(lhs.xmm, rhs.xmm));
    }

    [[nodiscard]] inline Float4Mask operator&(const Float4Mask& lhs, const Float4Mask& rhs)
    {
        return Float4Mask(_mm_and_ps(lhs.xmm, rhs.xmm));
    }

    [[nodiscard]] inline Float4Mask operator^(const Float4Mask& lhs, const Float4Mask& rhs)
    {
        return Float4Mask(_mm_xor_ps(lhs.xmm, rhs.xmm));
    }

    [[nodiscard]] inline Float4Mask operator~(const Float4Mask& lhs)
    {
        return Float4Mask(_mm_xor_ps(lhs.xmm, _mm_castsi128_ps(_mm_set1_epi32(0xFFFFFFFF))));
    }

    [[nodiscard]] inline Float4 operator-(const Float4& f)
    {
        return Float4(m128::neg(f.xmm));
    }

    [[nodiscard]] inline Float4 operator+(const Float4& lhs, const Float4& rhs)
    {
        return Float4(m128::add(lhs.xmm, rhs.xmm));
    }

    [[nodiscard]] inline Float4 operator-(const Float4& lhs, const Float4& rhs)
    {
        return Float4(m128::sub(lhs.xmm, rhs.xmm));
    }

    [[nodiscard]] inline Float4 operator*(const Float4& lhs, const Float4& rhs)
    {
        return Float4(m128::mul(lhs.xmm, rhs.xmm));
    }

    [[nodiscard]] inline Float4 operator/(const Float4& lhs, const Float4& rhs)
    {
        return Float4(m128::div(lhs.xmm, rhs.xmm));
    }

    [[nodiscard]] inline Float4Mask operator==(const Float4& lhs, const Float4& rhs) noexcept
    {
        return Float4Mask(m128::cmpeq(lhs.xmm, rhs.xmm));
    }

    [[nodiscard]] inline Float4Mask operator<(const Float4& lhs, const Float4& rhs) noexcept
    {
        return Float4Mask(m128::cmplt(lhs.xmm, rhs.xmm));
    }

    [[nodiscard]] inline Float4Mask operator>(const Float4& lhs, const Float4& rhs) noexcept
    {
        return Float4Mask(m128::cmplt(rhs.xmm, lhs.xmm));
    }

    [[nodiscard]] inline Float4Mask operator<=(const Float4& lhs, const Float4& rhs) noexcept
    {
        return Float4Mask(m128::cmpeq(lhs.xmm, rhs.xmm));
    }

    [[nodiscard]] inline Float4Mask operator>=(const Float4& lhs, const Float4& rhs) noexcept
    {
        return Float4Mask(m128::cmpeq(rhs.xmm, lhs.xmm));
    }

    [[nodiscard]] inline Float4 sqrt(const Float4& f)
    {
        return Float4(m128::sqrt(f.xmm));
    }

    [[nodiscard]] inline Float4 max(const Float4& a, const Float4& b)
    {
        return Float4(m128::max(a.xmm, b.xmm));
    }

    [[nodiscard]] inline Float4 min(const Float4& a, const Float4& b)
    {
        return Float4(m128::min(a.xmm, b.xmm));
    }

    [[nodiscard]] inline Float4 abs(const Float4& a)
    {
        return Float4(m128::abs(a.xmm));
    }
}

#include "m128/M128MemberSwizzleGeneratorUndef.h"
