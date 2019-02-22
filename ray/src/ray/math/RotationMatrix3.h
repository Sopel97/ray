#pragma once

#include "detail/M128Math.h"

#include "Vec3.h"

namespace ray
{
    template <typename T>
    struct RotationMatrix3;

    template <>
    struct alignas(alignof(__m128)) RotationMatrix3<float>
    {
        RotationMatrix3(
            const Normal3f& X,
            const Normal3f& Y,
            const Normal3f& Z
        ) :
            m_columns{ X, Y, Z }
        {
            transpose();
        }

        void transpose()
        {
            detail::transpose3(m_columns[0].xmm, m_columns[1].xmm, m_columns[2].xmm);
        }

        void invert()
        {
            // property of the rotation matrix
            transpose();
        }

        RotationMatrix3<float> inverse() const
        {
            RotationMatrix3<float> m = *this;
            m.invert();
            return m;
        }

        RotationMatrix3<float> transposed() const
        {
            RotationMatrix3<float> m = *this;
            m.transpose();
            return m;
        }

    private:
        // Hold the matrix transposed to allow multiplication
        // by a vector using multiplications instead of dot products.
        Normal3f m_columns[3];

        __m128 apply(const __m128& v) const
        {
            // the matrix is stored transposed
            __m128 v0 = _mm_shuffle_ps(v, v, _MM_SHUFFLE(0, 0, 0, 0));
            __m128 v1 = _mm_shuffle_ps(v, v, _MM_SHUFFLE(1, 1, 1, 1));
            __m128 v2 = _mm_shuffle_ps(v, v, _MM_SHUFFLE(2, 2, 2, 2));

            v0 = _mm_mul_ps(v0, m_columns[0].xmm);
            v1 = _mm_mul_ps(v1, m_columns[1].xmm);
            v2 = _mm_mul_ps(v2, m_columns[2].xmm);

            return _mm_add_ps(v0, _mm_add_ps(v1, v2));
        }

        friend Normal3f operator*(const RotationMatrix3<float>& m, const Normal3f& n)
        {
            return Vec3f(m.apply(n.xmm)).assumeNormalized();
        }

        friend Vec3f operator*(const RotationMatrix3<float>& m, const Vec3f& n)
        {
            return Vec3f(m.apply(n.xmm));
        }

        friend Point3f operator*(const RotationMatrix3<float>& m, const Point3f& n)
        {
            return Point3f(m.apply(n.xmm));
        }
    };

    using RotationMatrix3f = RotationMatrix3<float>;

}
