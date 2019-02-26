#pragma once

#include "detail/M128Math.h"
#include "detail/M128MatrixOperations.h"

#include "Vec3.h"

namespace ray
{
    template <typename T>
    struct Matrix4;

    template <>
    struct alignas(alignof(__m128)) Matrix4<float>
    {
        friend Matrix4<float> operator*(const Matrix4<float>& lhs, const Matrix4<float>& rhs)
        {
            Matrix4<float> ret{};
            detail::mulMat4Mat4(lhs.m_columns, rhs.m_columns, ret.m_columns);
            return ret;
        }

        friend Vec3<float> operator*(const Matrix4<float>& lhs, const Vec3<float>& rhs)
        {
            return Vec3<float>(detail::mulMat4Vec3(lhs.m_columns, rhs.xmm));
        }

        friend Point3<float> operator*(const Matrix4<float>& lhs, const Point3<float>& rhs)
        {
            return Point3<float>(detail::mulMat4Vec3(lhs.m_columns, rhs.xmm));
        }

        void transpose()
        {
            detail::transpose(m_columns);
        }

    protected:
        union
        {
            __m128 m_columns[4];
            // [col][row]
            float m_values[4][4];
        };

        Matrix4(__m128 c0, __m128 c1, __m128 c2, __m128 c3) :
            m_columns{ c0, c1, c2, c3 }
        {

        }

        Matrix4(__m128 c0, __m128 c1, __m128 c2) :
            m_columns{ c0, c1, c2, _mm_set_ps(1.0f, 0.0f, 0.0f, 0.0f) }
        {

        }

        // takes rows, transposes
        // will mostly be used in source with literals
        // so it is the most intuitive way
        Matrix4(
            float m00, float m10, float m20, float m30,
            float m01, float m11, float m21, float m31,
            float m02, float m12, float m22, float m32,
            float m03, float m13, float m23, float m33
        ) :
            m_values{
                m00, m01, m02, m03,
                m10, m11, m12, m13,
                m20, m21, m22, m23,
                m30, m31, m32, m33
            }
        {

        }

        Matrix4() :
            Matrix4(
                1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f
            )
        {

        };
    };

    static_assert(sizeof(Matrix4<float>) == sizeof(__m128) * 4);

    using Matrix4f = Matrix4<float>;
}
