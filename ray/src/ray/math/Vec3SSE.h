#pragma once

#include "Vec3.h"

#include <xmmintrin.h>
#include <smmintrin.h>

namespace ray
{
    inline float dot(__m128 a, __m128 b)
    {
        // mul first 3 components of v, sum it, and store in the first component, return first component
        return _mm_cvtss_f32(_mm_dp_ps(a, b, 0b0111'0001));
    }

    template <>
    struct Vec3Mask<float>
    {
        __m128 v;

        constexpr std::uint8_t packed() const
        {
            return _mm_movemask_ps(v);
        }
    };

    template <>
    struct Vec3<float>;

    template <>
    struct alignas(alignof(__m128)) Normal3<float>
    {
        union {
            struct {
                float x, y, z, _;
            };
            __m128 v;
        };

        constexpr static Normal3<float> xAxis()
        {
            return Normal3<float>(AssumeNormalized{}, 1, 0, 0);
        }

        constexpr static Normal3<float> yAxis()
        {
            return Normal3<float>(AssumeNormalized{}, 0, 1, 0);
        }

        constexpr static Normal3<float> zAxis()
        {
            return Normal3<float>(AssumeNormalized{}, 0, 0, 1);
        }

        constexpr Normal3() :
            x(1),
            y(0),
            z(0),
            _{}
        {
        }

        constexpr Normal3(AssumeNormalized, __m128 xyz_) :
            v(xyz_)
        {
        }

        constexpr Normal3(float x, float y, float z);

        constexpr explicit Normal3(const Vec3<float>& v);

        constexpr Normal3(AssumeNormalized, float x, float y, float z) :
            x(x),
            y(y),
            z(z),
            _{}
        {
        }

        constexpr Normal3(AssumeNormalized, const Vec3<float>& vec);

        constexpr Normal3(const Normal3<float>&) = default;
        constexpr Normal3(Normal3<float>&&) noexcept = default;

        constexpr Normal3<float>& operator=(const Normal3<float>&) = default;
        constexpr Normal3<float>& operator=(Normal3<float>&&) noexcept = default;

        constexpr explicit operator Vec3<float>() const;

        constexpr Vec3<float> reciprocal() const;
    };

    template <>
    struct alignas(alignof(__m128)) Vec3<float>
    {
        union {
            struct {
                float x, y, z, _;
            };
            __m128 v;
        };

        constexpr static Vec3<float> broadcast(float xyz)
        {
            return Vec3<float>(_mm_set1_ps(xyz));
        }
        constexpr Vec3() :
            v(_mm_set1_ps(0.0f))
        {
        }

        constexpr Vec3(__m128 xyz_) :
            v(xyz_)
        {
        }
        constexpr Vec3(float x, float y, float z) :
            x(x),
            y(y),
            z(z),
            _{}
        {

        }
        constexpr Vec3(const Vec3<float>&) = default;
        constexpr Vec3(Vec3<float>&&) noexcept = default;

        constexpr Vec3<float>& operator=(const Vec3<float>&) = default;
        constexpr Vec3<float>& operator=(Vec3<float>&&) noexcept = default;

        constexpr Vec3<float>& operator+=(const Vec3<float>& rhs)
        {
            v = _mm_add_ps(v, rhs.v);
            return *this;
        }

        constexpr Vec3<float>& operator-=(const Vec3<float>& rhs)
        {
            v = _mm_sub_ps(v, rhs.v);
            return *this;
        }

        constexpr Vec3<float>& operator*=(const Vec3<float>& rhs)
        {
            v = _mm_mul_ps(v, rhs.v);
            return *this;
        }

        constexpr Vec3<float>& operator/=(const Vec3<float>& rhs)
        {
            v = _mm_div_ps(v, rhs.v);
            return *this;
        }

        constexpr Vec3<float>& operator*=(float rhs)
        {
            return *this *= Vec3<float>::broadcast(rhs);
        }

        constexpr Vec3<float>& operator/=(float rhs)
        {
            return *this /= Vec3<float>::broadcast(rhs);
        }

        constexpr float length() const
        {
            using std::sqrt;
            return sqrt(lengthSqr());
        }

        float lengthSqr() const
        {
            return dot(v, v);
        }

        constexpr float invLength() const
        {
            return static_cast<float>(1) / length();
        }

        constexpr Vec3<float> reciprocal() const
        {
            return Vec3<float>(_mm_div_ps(_mm_set1_ps(1.0f), v));
        }

        constexpr void normalize()
        {
            *this *= invLength();
        }

        constexpr Normal3<float> assumeNormalized() const
        {
            return Normal3<float>(AssumeNormalized{}, *this);
        }

        constexpr Normal3<float> normalized() const
        {
            return Normal3<float>(AssumeNormalized{}, _mm_mul_ps(v, _mm_set1_ps(invLength())));
        }

        constexpr float max() const
        {
            using std::max;
            return max(max(x, y), z);
        }

        constexpr float min() const
        {
            using std::min;
            return min(min(x, y), z);
        }
    };

    constexpr Normal3<float>::Normal3(float x, float y, float z) :
        Normal3(Vec3<float>(x, y, z).normalized())
    {
    }

    constexpr Normal3<float>::Normal3(const Vec3<float>& v) :
        Normal3(v.normalized())
    {
    }

    constexpr Normal3<float>::Normal3(AssumeNormalized, const Vec3<float>& vec) :
        Normal3(AssumeNormalized{}, vec.v)
    {
    }

    constexpr Normal3<float>::operator Vec3<float>() const
    {
        return Vec3<float>(v);
    }

    constexpr Vec3<float> Normal3<float>::reciprocal() const
    {
        return Vec3<float>(v).reciprocal();
    }

    template <>
    struct alignas(alignof(__m128)) Point3<float>
    {
        union {
            struct {
                float x, y, z, _;
            };
            __m128 v;
        };

        constexpr static Point3<float> broadcast(float xyz)
        {
            return Point3<float>(_mm_set1_ps(xyz));
        }
        constexpr static Point3<float> origin()
        {
            return Point3<float>{};
        }
        constexpr Point3(__m128 xyz_) :
            v(xyz_)
        {
        }
        constexpr Point3() :
            v(_mm_set1_ps(0.0f))
        {
        }

        constexpr Point3(float x, float y, float z) :
            x(x),
            y(y),
            z(z),
            _{}
        {

        }
        constexpr Point3(const Point3<float>&) = default;
        constexpr Point3(Point3<float>&&) noexcept = default;

        constexpr Point3<float>& operator=(const Point3<float>&) = default;
        constexpr Point3<float>& operator=(Point3<float>&&) noexcept = default;

        constexpr Point3<float>& operator+=(const Vec3<float>& rhs)
        {
            v = _mm_add_ps(v, rhs.v);
            return *this;
        }

        constexpr Point3<float>& operator-=(const Vec3<float>& rhs)
        {
            v = _mm_sub_ps(v, rhs.v);
            return *this;
        }

        constexpr explicit operator Vec3<float>() const
        {
            return Vec3<float>(v);
        }
    };

    // Operations

    inline Vec3<float> operator-(const Point3<float>& lhs, const Point3<float>& rhs)
    {
        return Vec3<float>(_mm_sub_ps(lhs.v, rhs.v));
    }

    inline Point3<float> operator-(const Point3<float>& lhs, const Vec3<float>& rhs)
    {
        return Point3<float>(_mm_sub_ps(lhs.v, rhs.v));
    }

    inline Point3<float> operator+(const Point3<float>& lhs, const Vec3<float>& rhs)
    {
        return Point3<float>(_mm_add_ps(lhs.v, rhs.v));
    }

    inline Vec3<float> operator+(const Vec3<float>& lhs, const Vec3<float>& rhs)
    {
        return Vec3<float>(_mm_add_ps(lhs.v, rhs.v));
    }

    inline Vec3<float> operator-(const Vec3<float>& lhs, const Vec3<float>& rhs)
    {
        return Vec3<float>(_mm_sub_ps(lhs.v, rhs.v));
    }

    inline Vec3<float> operator*(const Vec3<float>& lhs, const Vec3<float>& rhs)
    {
        return Vec3<float>(_mm_mul_ps(lhs.v, rhs.v));
    }

    inline Vec3<float> operator/(const Vec3<float>& lhs, const Vec3<float>& rhs)
    {
        return Vec3<float>(_mm_div_ps(lhs.v, rhs.v));
    }

    inline Vec3<float> operator*(const Vec3<float>& lhs, float rhs)
    {
        return Vec3<float>(_mm_mul_ps(lhs.v, _mm_set1_ps(rhs)));
    }

    inline Vec3<float> operator*(const Normal3<float>& lhs, float rhs)
    {
        return Vec3<float>(_mm_mul_ps(lhs.v, _mm_set1_ps(rhs)));
    }

    inline Vec3<float> operator*(float lhs, const Vec3<float>& rhs)
    {
        return Vec3<float>(_mm_mul_ps(rhs.v, _mm_set1_ps(lhs)));
    }

    inline Vec3<float> operator/(const Vec3<float>& lhs, float rhs)
    {
        return Vec3<float>(_mm_div_ps(lhs.v, _mm_set1_ps(rhs)));
    }

    inline Vec3<float> operator*(float lhs, const Normal3<float>& rhs)
    {
        return Vec3<float>(_mm_mul_ps(rhs.v, _mm_set1_ps(lhs)));
    }

    inline Vec3<float> operator/(const Normal3<float>& lhs, float rhs)
    {
        return Vec3<float>(_mm_div_ps(lhs.v, _mm_set1_ps(rhs)));
    }

    inline Vec3Mask<float> operator<(const Vec3<float> & lhs, const Vec3<float>& rhs)
    {
        return Vec3Mask<float>{_mm_cmplt_ps(lhs.v, rhs.v)};
    }

    inline Vec3Mask<float> operator<(const Normal3<float>& lhs, const Vec3<float>& rhs)
    {
        return Vec3Mask<float>{_mm_cmplt_ps(lhs.v, rhs.v)};
    }

    inline Vec3Mask<float> operator<(const Vec3<float>& lhs, float rhs)
    {
        return Vec3Mask<float>{_mm_cmplt_ps(lhs.v, _mm_set1_ps(rhs))};
    }

    inline Vec3Mask<float> operator<(const Normal3<float>& lhs, float rhs)
    {
        return Vec3Mask<float>{_mm_cmplt_ps(lhs.v, _mm_set1_ps(rhs))};
    }

    inline float dot(const Vec3<float>& lhs, const Vec3<float>& rhs)
    {
        return dot(lhs.v, rhs.v);
    }

    inline float dot(const Normal3<float>& lhs, const Vec3<float>& rhs)
    {
        return dot(lhs.v, rhs.v);
    }

    inline float dot(const Vec3<float>& lhs, const Normal3<float>& rhs)
    {
        return dot(lhs.v, rhs.v);
    }

    inline float dot(const Normal3<float>& lhs, const Normal3<float>& rhs)
    {
        return dot(lhs.v, rhs.v);
    }

    inline __m128 cross(__m128 a, __m128 b)
    {
        // http://threadlocalmutex.com/?p=8
        __m128 a_yzx = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1));
        __m128 b_yzx = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 0, 2, 1));
        __m128 c = _mm_sub_ps(_mm_mul_ps(a, b_yzx), _mm_mul_ps(a_yzx, b));
        return _mm_shuffle_ps(c, c, _MM_SHUFFLE(3, 0, 2, 1));
    }

    inline Vec3<float> cross(const Vec3<float>& lhs, const Vec3<float>& rhs)
    {
        return Vec3<float>(cross(lhs.v, rhs.v));
    }

    inline Vec3<float> cross(const Normal3<float>& lhs, const Vec3<float>& rhs)
    {
        return Vec3<float>(cross(lhs.v, rhs.v));
    }

    inline Vec3<float> cross(const Vec3<float>& lhs, const Normal3<float>& rhs)
    {
        return Vec3<float>(cross(lhs.v, rhs.v));
    }

    inline Vec3<float> cross(const Normal3<float>& lhs, const Normal3<float>& rhs)
    {
        return Vec3<float>(cross(lhs.v, rhs.v));
    }

    constexpr Vec3<float> min(const Vec3<float>& lhs, const Vec3<float>& rhs)
    {
        return Vec3<float>(_mm_min_ps(lhs.v, rhs.v));
    }

    constexpr Vec3<float> max(const Vec3<float>& lhs, const Vec3<float>& rhs)
    {
        return Vec3<float>(_mm_max_ps(lhs.v, rhs.v));
    }

    constexpr Normal3<float> operator-(const Normal3<float>& vec)
    {
        return Normal3<float>(AssumeNormalized{}, _mm_sub_ps(_mm_set1_ps(0.0f), vec.v));
    }

    constexpr Vec3<float> operator-(const Vec3<float>& vec)
    {
        return Vec3<float>(_mm_sub_ps(_mm_set1_ps(0.0f), vec.v));
    }

    constexpr Point3<float> operator-(const Point3<float>& vec)
    {
        return Point3<float>(_mm_sub_ps(_mm_set1_ps(0.0f), vec.v));
    }
}
