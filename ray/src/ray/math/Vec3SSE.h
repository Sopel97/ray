#pragma once

#include "Vec3.h"

#include "detail/M128Math.h"

#include <xmmintrin.h>
#include <smmintrin.h>

namespace ray
{
    template <>
    struct Vec3<float>;

    template <>
    struct Vec3Mask<float>
    {
        __m128 xmm;

        explicit Vec3Mask(__m128 xmm) :
            xmm(xmm)
        {
        }

        std::uint8_t packed() const
        {
            return _mm_movemask_ps(xmm);
        }
    };

    template <>
    struct alignas(alignof(__m128)) Normal3<float>
    {
        union {
            struct {
                float x, y, z, _;
            };
            __m128 xmm;
        };

        static Normal3<float> xAxis()
        {
            return Normal3<float>(AssumeNormalized{}, 1, 0, 0);
        }

        static Normal3<float> yAxis()
        {
            return Normal3<float>(AssumeNormalized{}, 0, 1, 0);
        }

        static Normal3<float> zAxis()
        {
            return Normal3<float>(AssumeNormalized{}, 0, 0, 1);
        }

        Normal3() :
            xmm(_mm_set_ps(0.0f, 0.0f, 0.0f, 1.0f))
        {
        }

        Normal3(AssumeNormalized, __m128 xmm) :
            xmm(xmm)
        {
        }

        Normal3(float x, float y, float z);
        Normal3(AssumeNormalized, const Vec3<float>& vec);

        Normal3(AssumeNormalized, float x, float y, float z) :
            xmm(_mm_set_ps(0.0f, z, y, x))
        {
        }

        Normal3(const Normal3<float>&) = default;
        Normal3(Normal3<float>&&) noexcept = default;

        Normal3<float>& operator=(const Normal3<float>&) = default;
        Normal3<float>& operator=(Normal3<float>&&) noexcept = default;

        explicit operator Vec3<float>() const;

        void negate(const Vec3Mask<float>& mask)
        {
            xmm = detail::neg(xmm, mask.xmm);
        }
    };
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

        static Vec3<float> broadcast(float xyz)
        {
            return Vec3<float>(_mm_set1_ps(xyz));
        }
        static Vec3<float> blend(float a, float b, const Vec3Mask<float>& mask)
        {
            return Vec3<float>(detail::blend(a, b, mask.xmm));
        }

        Vec3() :
            xmm(_mm_set1_ps(0.0f))
        {
        }

        explicit Vec3(__m128 xyz_) :
            xmm(xyz_)
        {
        }
        Vec3(float x, float y, float z) :
            xmm(_mm_set_ps(0.0f, z, y, x))
        {

        }
        Vec3(const Vec3<float>&) = default;
        Vec3(Vec3<float>&&) noexcept = default;

        Vec3<float>& operator=(const Vec3<float>&) = default;
        Vec3<float>& operator=(Vec3<float>&&) noexcept = default;

        Vec3<float>& operator+=(const Vec3<float>& rhs)
        {
            xmm = detail::add(xmm, rhs.xmm);
            return *this;
        }

        Vec3<float>& operator-=(const Vec3<float>& rhs)
        {
            xmm = detail::sub(xmm, rhs.xmm);
            return *this;
        }

        Vec3<float>& operator*=(const Vec3<float>& rhs)
        {
            xmm = detail::mul(xmm, rhs.xmm);
            return *this;
        }

        Vec3<float>& operator/=(const Vec3<float>& rhs)
        {
            xmm = detail::div(xmm, rhs.xmm);
            return *this;
        }

        Vec3<float>& operator*=(float rhs)
        {
            xmm = detail::mul(xmm, rhs);
            return *this;
        }

        Vec3<float>& operator/=(float rhs)
        {
            xmm = detail::div(xmm, rhs);
            return *this;
        }

        float length() const
        {
            using std::sqrt;
            return sqrt(lengthSqr());
        }

        float lengthSqr() const
        {
            return detail::dot(xmm, xmm);
        }

        float invLength() const
        {
            return static_cast<float>(1) / length();
        }

        void normalize()
        {
            *this *= invLength();
        }

        Normal3<float> assumeNormalized() const
        {
            return Normal3<float>(AssumeNormalized{}, *this);
        }

        Normal3<float> normalized() const
        {
            return Normal3<float>(AssumeNormalized{}, detail::mul(xmm, invLength()));
        }

        float max() const
        {
            using std::max;
            return max(max(x, y), z);
        }

        float min() const
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

        static Point3<float> broadcast(float xyz)
        {
            return Point3<float>(_mm_set1_ps(xyz));
        }
        static Point3<float> origin()
        {
            return Point3<float>{};
        }
        explicit Point3(__m128 xyz_) :
            xmm(xyz_)
        {
        }
        Point3() :
            xmm(_mm_set1_ps(0.0f))
        {
        }

        Point3(float x, float y, float z) :
            xmm(_mm_set_ps(0.0f, z, y, x))
        {

        }
        Point3(const Point3<float>&) = default;
        Point3(Point3<float>&&) noexcept = default;

        Point3<float>& operator=(const Point3<float>&) = default;
        Point3<float>& operator=(Point3<float>&&) noexcept = default;

        Point3<float>& operator+=(const Vec3<float>& rhs)
        {
            xmm = detail::add(xmm, rhs.xmm);
            return *this;
        }

        Point3<float>& operator-=(const Vec3<float>& rhs)
        {
            xmm = detail::sub(xmm, rhs.xmm);
            return *this;
        }

        explicit operator Vec3<float>() const
        {
            return Vec3<float>(xmm);
        }
    };
    static_assert(sizeof(Point3f) == sizeof(__m128));

    inline Normal3<float>::Normal3(float x, float y, float z) :
        Normal3(Vec3<float>(x, y, z).normalized())
    {
    }

    inline Normal3<float>::Normal3(AssumeNormalized, const Vec3<float>& vec) :
        Normal3(AssumeNormalized{}, vec.xmm)
    {
    }

    inline Normal3<float>::operator Vec3<float>() const
    {
        return Vec3<float>(xmm);
    }

    // Operations

    inline Vec3Mask<float> operator|(const Vec3Mask<float>& lhs, const Vec3Mask<float>& rhs)
    {
        return Vec3Mask<float>(_mm_or_ps(lhs.xmm, rhs.xmm));
    }

    inline Vec3Mask<float> operator&(const Vec3Mask<float>& lhs, const Vec3Mask<float>& rhs)
    {
        return Vec3Mask<float>(_mm_and_ps(lhs.xmm, rhs.xmm));
    }

    inline Vec3Mask<float> operator^(const Vec3Mask<float>& lhs, const Vec3Mask<float>& rhs)
    {
        return Vec3Mask<float>(_mm_xor_ps(lhs.xmm, rhs.xmm));
    }

    inline Vec3Mask<float> operator~(const Vec3Mask<float>& lhs)
    {
        return Vec3Mask<float>(_mm_xor_ps(lhs.xmm, _mm_castsi128_ps(_mm_set1_epi32(0xFFFFFFFF))));
    }

    inline Vec3<float> operator-(const Point3<float>& lhs, const Point3<float>& rhs)
    {
        return Vec3<float>(detail::sub(lhs.xmm, rhs.xmm));
    }

    inline Point3<float> operator-(const Point3<float>& lhs, const Vec3<float>& rhs)
    {
        return Point3<float>(detail::sub(lhs.xmm, rhs.xmm));
    }

    inline Vec3<float> operator-(const Vec3<float>& lhs, const Vec3<float>& rhs)
    {
        return Vec3<float>(detail::sub(lhs.xmm, rhs.xmm));
    }


    inline Point3<float> operator+(const Point3<float>& lhs, const Vec3<float>& rhs)
    {
        return Point3<float>(detail::add(lhs.xmm, rhs.xmm));
    }

    inline Vec3<float> operator+(const Vec3<float>& lhs, const Vec3<float>& rhs)
    {
        return Vec3<float>(detail::add(lhs.xmm, rhs.xmm));
    }


    inline Vec3<float> operator*(const Vec3<float>& lhs, const Vec3<float>& rhs)
    {
        return Vec3<float>(detail::mul(lhs.xmm, rhs.xmm));
    }

    inline Vec3<float> operator*(const Vec3<float>& lhs, float rhs)
    {
        return Vec3<float>(detail::mul(lhs.xmm, rhs));
    }

    inline Vec3<float> operator*(const Normal3<float>& lhs, float rhs)
    {
        return Vec3<float>(detail::mul(lhs.xmm, rhs));
    }

    inline Vec3<float> operator*(float lhs, const Vec3<float>& rhs)
    {
        return Vec3<float>(detail::mul(lhs, rhs.xmm));
    }

    inline Vec3<float> operator*(float lhs, const Normal3<float>& rhs)
    {
        return Vec3<float>(detail::mul(lhs, rhs.xmm));
    }


    inline Vec3<float> operator/(const Vec3<float>& lhs, float rhs)
    {
        return Vec3<float>(detail::div(lhs.xmm, rhs));
    }

    inline Vec3<float> operator/(const Vec3<float>& lhs, const Vec3<float>& rhs)
    {
        return Vec3<float>(detail::div(lhs.xmm, rhs.xmm));
    }

    inline Vec3<float> operator/(const Normal3<float>& lhs, float rhs)
    {
        return Vec3<float>(detail::div(lhs.xmm, rhs));
    }

    inline Vec3<float> operator/(float lhs, const Vec3<float>& rhs)
    {
        return Vec3<float>(detail::div(lhs, rhs.xmm));
    }

    inline Vec3<float> operator/(float lhs, const Normal3<float>& rhs)
    {
        return Vec3<float>(detail::div(lhs, rhs.xmm));
    }


    inline Vec3Mask<float> operator==(const Vec3<float> & lhs, const Vec3<float>& rhs)
    {
        return Vec3Mask<float>{detail::cmpeq(lhs.xmm, rhs.xmm)};
    }

    inline Vec3Mask<float> operator==(const Normal3<float>& lhs, const Normal3<float>& rhs)
    {
        return Vec3Mask<float>{detail::cmpeq(lhs.xmm, rhs.xmm)};
    }

    inline Vec3Mask<float> operator==(const Point3<float>& lhs, const Point3<float>& rhs)
    {
        return Vec3Mask<float>{detail::cmpeq(lhs.xmm, rhs.xmm)};
    }

    inline Vec3Mask<float> operator==(const Vec3<float>& lhs, float rhs)
    {
        return Vec3Mask<float>{detail::cmpeq(lhs.xmm, rhs)};
    }

    inline Vec3Mask<float> operator==(const Normal3<float>& lhs, float rhs)
    {
        return Vec3Mask<float>{detail::cmpeq(lhs.xmm, rhs)};
    }

    inline Vec3Mask<float> operator==(const Point3<float>& lhs, float rhs)
    {
        return Vec3Mask<float>{detail::cmpeq(lhs.xmm, rhs)};
    }


    inline Vec3Mask<float> operator<(const Vec3<float> & lhs, const Vec3<float>& rhs)
    {
        return Vec3Mask<float>{detail::cmplt(lhs.xmm, rhs.xmm)};
    }

    inline Vec3Mask<float> operator<(const Normal3<float>& lhs, const Normal3<float>& rhs)
    {
        return Vec3Mask<float>{detail::cmplt(lhs.xmm, rhs.xmm)};
    }

    inline Vec3Mask<float> operator<(const Point3<float>& lhs, const Point3<float>& rhs)
    {
        return Vec3Mask<float>{detail::cmplt(lhs.xmm, rhs.xmm)};
    }

    inline Vec3Mask<float> operator<(const Vec3<float>& lhs, float rhs)
    {
        return Vec3Mask<float>{detail::cmplt(lhs.xmm, rhs)};
    }

    inline Vec3Mask<float> operator<(const Normal3<float>& lhs, float rhs)
    {
        return Vec3Mask<float>{detail::cmplt(lhs.xmm, rhs)};
    }

    inline Vec3Mask<float> operator<(const Point3<float>& lhs, float rhs)
    {
        return Vec3Mask<float>{detail::cmplt(lhs.xmm, rhs)};
    }


    inline Vec3Mask<float> operator<=(const Vec3<float> & lhs, const Vec3<float>& rhs)
    {
        return Vec3Mask<float>{detail::cmple(lhs.xmm, rhs.xmm)};
    }

    inline Vec3Mask<float> operator<=(const Normal3<float>& lhs, const Normal3<float>& rhs)
    {
        return Vec3Mask<float>{detail::cmple(lhs.xmm, rhs.xmm)};
    }

    inline Vec3Mask<float> operator<=(const Point3<float>& lhs, const Point3<float>& rhs)
    {
        return Vec3Mask<float>{detail::cmple(lhs.xmm, rhs.xmm)};
    }

    inline Vec3Mask<float> operator<=(const Vec3<float>& lhs, float rhs)
    {
        return Vec3Mask<float>{detail::cmple(lhs.xmm, rhs)};
    }

    inline Vec3Mask<float> operator<=(const Normal3<float>& lhs, float rhs)
    {
        return Vec3Mask<float>{detail::cmple(lhs.xmm, rhs)};
    }

    inline Vec3Mask<float> operator<=(const Point3<float>& lhs, float rhs)
    {
        return Vec3Mask<float>{detail::cmple(lhs.xmm, rhs)};
    }


    inline Vec3<float> rcp(const Vec3<float>& v)
    {
        return 1.0f / v;
    }

    inline Vec3<float> rcp(const Normal3<float>& v)
    {
        return 1.0f / v;
    }


    inline Vec3<float> abs(const Vec3<float>& v)
    {
        return Vec3<float>(detail::abs(v.xmm));
    }

    inline Normal3<float> abs(const Normal3<float>& v)
    {
        return Normal3<float>(AssumeNormalized{}, detail::abs(v.xmm));
    }

    inline Point3<float> abs(const Point3<float>& v)
    {
        return Point3<float>(detail::abs(v.xmm));
    }


    inline float dot(const Vec3<float>& lhs, const Vec3<float>& rhs)
    {
        return detail::dot(lhs.xmm, rhs.xmm);
    }

    inline float dot(const Normal3<float>& lhs, const Vec3<float>& rhs)
    {
        return detail::dot(lhs.xmm, rhs.xmm);
    }

    inline float dot(const Vec3<float>& lhs, const Normal3<float>& rhs)
    {
        return detail::dot(lhs.xmm, rhs.xmm);
    }

    inline float dot(const Normal3<float>& lhs, const Normal3<float>& rhs)
    {
        return detail::dot(lhs.xmm, rhs.xmm);
    }


    inline Vec3<float> cross(const Vec3<float>& lhs, const Vec3<float>& rhs)
    {
        return Vec3<float>(detail::cross(lhs.xmm, rhs.xmm));
    }

    inline Vec3<float> cross(const Normal3<float>& lhs, const Vec3<float>& rhs)
    {
        return Vec3<float>(detail::cross(lhs.xmm, rhs.xmm));
    }

    inline Vec3<float> cross(const Vec3<float>& lhs, const Normal3<float>& rhs)
    {
        return Vec3<float>(detail::cross(lhs.xmm, rhs.xmm));
    }

    inline Vec3<float> cross(const Normal3<float>& lhs, const Normal3<float>& rhs)
    {
        return Vec3<float>(detail::cross(lhs.xmm, rhs.xmm));
    }


    inline Vec3<float> min(const Vec3<float>& lhs, const Vec3<float>& rhs)
    {
        return Vec3<float>(detail::min(lhs.xmm, rhs.xmm));
    }

    inline Point3<float> min(const Point3<float>& lhs, const Point3<float>& rhs)
    {
        return Point3<float>(detail::min(lhs.xmm, rhs.xmm));
    }


    inline Point3<float> max(const Point3<float>& lhs, const Point3<float>& rhs)
    {
        return Point3<float>(detail::max(lhs.xmm, rhs.xmm));
    }

    inline Vec3<float> max(const Vec3<float>& lhs, const Vec3<float>& rhs)
    {
        return Vec3<float>(detail::max(lhs.xmm, rhs.xmm));
    }


    inline Vec3<float> sqrt(const Vec3<float>& lhs)
    {
        return Vec3<float>(detail::sqrt(lhs.xmm));
    }


    inline Normal3<float> operator-(const Normal3<float>& vec)
    {
        return Normal3<float>(AssumeNormalized{}, detail::neg(vec.xmm));
    }

    inline Vec3<float> operator-(const Vec3<float>& vec)
    {
        return Vec3<float>(detail::neg(vec.xmm));
    }

    inline Point3<float> operator-(const Point3<float>& vec)
    {
        return Point3<float>(detail::neg(vec.xmm));
    }
}
