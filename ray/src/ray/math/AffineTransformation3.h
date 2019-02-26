#pragma once

#include "detail/M128Math.h"

#include "Vec3.h"

#include <string>

namespace ray
{
    template <typename T>
    struct AffineTransformationMatrix3;

    template <>
    struct alignas(alignof(__m128)) AffineTransformationMatrix3<float>
    {
        AffineTransformationMatrix3(
            const Vec3f& X,
            const Vec3f& Y,
            const Vec3f& Z,
            const Vec3f& T
        ) :
            m_columns{ X, Y, Z, T }
        {
            detail::transpose3(m_columns[0].xmm, m_columns[1].xmm, m_columns[2].xmm);
        }

        void transpose()
        {
            detail::transpose(m_columns[0].xmm, m_columns[1].xmm, m_columns[2].xmm, m_columns[3].xmm);
        }

        void invert()
        {
            // column-major
            // X0 X1 X2 T0
            // Y0 Y1 Y2 T1
            // Z0 Z1 Z2 T2
            //
            // R = 3x3 rotation matrix
            // T = translation
            //
            // R | T
            // 0 | 1
            //
            // inverted:
            //
            // (1/(X^2))*X0 (1/(Y^2))*Y0 (1/(Z^2))*Z0 T0'
            // (1/(X^2))*X1 (1/(Y^2))*Y1 (1/(Z^2))*Z1 T1'
            // (1/(X^2))*X2 (1/(Y^2))*Y2 (1/(Z^2))*Z2 T2'
            // where T' = (R^-1)*(-T)
            //
            // R^-1 | (R^-1)*(-T)
            //   0  |    1
            //
            // but we don't care what the last row holds because it's always ignored in the computations

            __m128 c0 = m_columns[0].xmm;
            __m128 c1 = m_columns[1].xmm;
            __m128 c2 = m_columns[2].xmm;

            // before inverting R compute inverse scales
            __m128 scales =
                _mm_add_ps(
                    _mm_add_ps(
                        _mm_mul_ps(c0, c0),
                        _mm_mul_ps(c1, c1)
                    ),
                    _mm_mul_ps(c2, c2)
                );

            __m128 invScales = _mm_div_ps(_mm_set1_ps(1.0f), scales);

            // apply scaling
            c0 = _mm_mul_ps(c0, invScales);
            c1 = _mm_mul_ps(c1, invScales);
            c2 = _mm_mul_ps(c2, invScales);

            // R^-1 (with rescaling above)
            detail::transpose3(c0, c1, c2);

            __m128 c3 = m_columns[3].xmm;

            // multiply (R^-1) * T
            __m128 v0 = _mm_shuffle_ps(c3, c3, _MM_SHUFFLE(0, 0, 0, 0));
            __m128 v1 = _mm_shuffle_ps(c3, c3, _MM_SHUFFLE(1, 1, 1, 1));
            __m128 v2 = _mm_shuffle_ps(c3, c3, _MM_SHUFFLE(2, 2, 2, 2));

            v0 = _mm_mul_ps(v0, c0);
            v1 = _mm_mul_ps(v1, c1);
            v2 = _mm_mul_ps(v2, c2);

            // store T'
            c3 = _mm_add_ps(v0, _mm_add_ps(v1, v2));

            m_columns[0].xmm = c0;
            m_columns[1].xmm = c1;
            m_columns[2].xmm = c2;
            m_columns[3].xmm = detail::neg(c3);
        }

        AffineTransformationMatrix3<float> inverse() const
        {
            AffineTransformationMatrix3<float> m = *this;
            m.invert();
            return m;
        }

        AffineTransformationMatrix3<float> transposed() const
        {
            AffineTransformationMatrix3<float> m = *this;
            m.transpose();
            return m;
        }

    private:
        // Hold the matrix transposed to allow multiplication
        // by a vector using multiplications instead of dot products.
        Vec3f m_columns[4];

        __m128 apply(const __m128& v) const
        {
            // the matrix is stored transposed
            __m128 v0 = _mm_shuffle_ps(v, v, _MM_SHUFFLE(0, 0, 0, 0));
            __m128 v1 = _mm_shuffle_ps(v, v, _MM_SHUFFLE(1, 1, 1, 1));
            __m128 v2 = _mm_shuffle_ps(v, v, _MM_SHUFFLE(2, 2, 2, 2));

            v0 = _mm_mul_ps(v0, m_columns[0].xmm);
            v1 = _mm_mul_ps(v1, m_columns[1].xmm);
            v2 = _mm_mul_ps(v2, m_columns[2].xmm);

            return _mm_add_ps(_mm_add_ps(v0, v1), _mm_add_ps(v2, m_columns[3].xmm));
        }

        friend Vec3f operator*(const AffineTransformationMatrix3<float>& m, const Normal3f& n)
        {
            return Vec3f(m.apply(n.xmm));
        }

        friend Vec3f operator*(const AffineTransformationMatrix3<float>& m, const Vec3f& n)
        {
            return Vec3f(m.apply(n.xmm));
        }

        friend Point3f operator*(const AffineTransformationMatrix3<float>& m, const Point3f& n)
        {
            return Point3f(m.apply(n.xmm));
        }
    };

    using AffineTransformationMatrix3f = AffineTransformationMatrix3<float>;
}
