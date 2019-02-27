#pragma once

#include "detail/M128Math.h"
#include "detail/M128MatrixOperations.h"

#include "Quat3.h"
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

        void transpose3()
        {
            detail::transpose3(m_columns);
        }

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

        Matrix4(float xs, float ys, float zs) :
            Matrix4(
                xs, 0.0f, 0.0f, 0.0f,
                0.0f, ys, 0.0f, 0.0f,
                0.0f, 0.0f, zs, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f
            )
        {
        }

        Matrix4(float xs, float ys, float zs, const Vec3<float>& t) :
            Matrix4(
                xs, 0.0f, 0.0f, 0.0f,
                0.0f, ys, 0.0f, 0.0f,
                0.0f, 0.0f, zs, 0.0f,
                t.x, t.y, t.z, 1.0f
            )
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
        }

        explicit Matrix4(const Vec3<float>& t) :
            Matrix4(
                1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f, 0.0f,
                t.x, t.y, t.z, 1.0f
            )
        {
        }

        explicit Matrix4(const OrthonormalBasis3<float>& basis) :
            Matrix4(
                detail::truncate3(basis.x().xmm),
                detail::truncate3(basis.y().xmm),
                detail::truncate3(basis.z().xmm)
            )
        {
            transpose3();
        }

        explicit Matrix4(const Basis3<float>& basis) :
            Matrix4(
                detail::truncate3(basis.x().xmm),
                detail::truncate3(basis.y().xmm),
                detail::truncate3(basis.z().xmm)
            )
        {
            transpose3();
        }

        Matrix4(const OrthonormalBasis3<float>& basis, const Vec3<float>& t) :
            Matrix4(
                detail::truncate3(basis.x().xmm),
                detail::truncate3(basis.y().xmm),
                detail::truncate3(basis.z().xmm),
                _mm_set_ps(1.0f, t.z, t.y, t.x)
            )
        {
            transpose3();
        }

        Matrix4(const Basis3<float>& basis, const Vec3<float>& t) :
            Matrix4(
                detail::truncate3(basis.x().xmm),
                detail::truncate3(basis.y().xmm),
                detail::truncate3(basis.z().xmm),
                _mm_set_ps(1.0f, t.z, t.y, t.x)
            )
        {
            transpose3();
        }

        explicit Matrix4(const Quat3<float>& q) :
            Matrix4()
        {
            const float x2 = q.x*q.x;
            const float y2 = q.y*q.y;
            const float z2 = q.z*q.z;

            // col 0
            m_values[0][0] = 1.0f - 2.0f * (y2 + z2);
            m_values[0][1] = 2.0f * (q.x*q.y + q.z*q.w);
            m_values[0][2] = 2.0f * (q.x*q.z - q.y*q.w);

            // col 1
            m_values[1][0] = 2.0f * (q.x*q.y - q.z*q.w);
            m_values[1][1] = 1.0f - 2.0f * (x2 + z2);
            m_values[1][2] = 2.0f * (q.y*q.z + q.x*q.w);

            // col 2
            m_values[2][0] = 2.0f * (q.x*q.z + q.y*q.w);
            m_values[2][1] = 2.0f * (q.y*q.z - q.x*q.w);
            m_values[2][2] = 1.0f - 2.0f * (x2 + y2);
        }

        Matrix4(const Quat3<float>& q, const Vec3<float>& t) :
            Matrix4(q)
        {
            m_values[3][0] = t.x;
            m_values[3][1] = t.y;
            m_values[3][2] = t.z;
        }

        Matrix4(const Quat3<float>& q, const Point3<float>& origin) :
            Matrix4(q, Vec3<float>(origin))
        {
            // (TR)T^-1
            // TR done in the constructor

            // R | T     1 | -T     R | -RT+T
            // -----  *  ------  =  ---------
            // 0 | 1     0 | 1      0 |   1
            const Vec3<float> t(detail::sub(origin.xmm, detail::mulMat3Vec3(m_columns, origin.xmm)));
            m_values[3][0] = t.x;
            m_values[3][1] = t.y;
            m_values[3][2] = t.z;
        }
    };

    static_assert(sizeof(Matrix4<float>) == sizeof(__m128) * 4);

    using Matrix4f = Matrix4<float>;
}
