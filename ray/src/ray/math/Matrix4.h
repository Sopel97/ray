#pragma once

#include "m128/M128Math.h"
#include "m128/M128MatrixOperations.h"

#include "Quat4.h"
#include "Vec3.h"
#include "ViewingFrustum3.h"

#include <iostream>

namespace ray
{
    template <typename T>
    struct Matrix4;

    template <>
    struct alignas(alignof(__m128)) Matrix4<float>
    {
        // takes rows, transposes
        // will mostly be used in source with literals
        // so it is the most intuitive way
        Matrix4(
            float m00, float m10, float m20, float m30,
            float m01, float m11, float m21, float m31,
            float m02, float m12, float m22, float m32,
            float m03, float m13, float m23, float m33
        ) noexcept :
            m_values{
                m00, m01, m02, m03,
                m10, m11, m12, m13,
                m20, m21, m22, m23,
                m30, m31, m32, m33
        }
        {
        }

        Matrix4(const ViewingFrustum3<float>& f) noexcept :
            Matrix4()
        {
            // http://learnwebgl.brown37.net/lib/learn_webgl_matrix.js

            const float near = f.near;
            const float far = f.far;
            const float top = near * (f.fovy * 0.5f).tan();
            const float bottom = -top;
            const float right = top * f.aspect;
            const float left = -right;

            const float sx = 2.0f * near / (right - left);
            const float sy = 2.0f * near / (top - bottom);

            const float c2 = -(far + near) / (far - near);
            const float c1 = 2.0f * near * far / (near - far);

            const float tx = -near * (left + right) / (right - left);
            const float ty = -near * (bottom + top) / (top - bottom);

            m_values[0][0] = sx;

            m_values[1][1] = sy;

            m_values[2][2] = c2;
            m_values[2][3] = -1.0f;

            m_values[3][0] = tx;
            m_values[3][1] = ty;
            m_values[3][2] = c1;
            m_values[3][3] = 0.0f;
        }

        [[nodiscard]] friend Matrix4<float> operator*(const Matrix4<float>& lhs, const Matrix4<float>& rhs)
        {
            Matrix4<float> ret{};
            m128::mulMat4Mat4(lhs.m_columns, rhs.m_columns, ret.m_columns);
            return ret;
        }

        [[nodiscard]] friend Vec3<float> operator*(const Matrix4<float>& lhs, const Vec3<float>& rhs)
        {
            return Vec3<float>(m128::mulMat4Vec3(lhs.m_columns, rhs.xmm));
        }

        [[nodiscard]] friend Point3<float> operator*(const Matrix4<float>& lhs, const Point3<float>& rhs)
        {
            return Point3<float>(m128::mulMat4Vec3(lhs.m_columns, rhs.xmm));
        }

        [[nodiscard]] bool isAlmostIdentity() const
        {
            static constexpr float eps = 1e-5;
            for (int r = 0; r < 4; ++r)
            {
                for (int c = 0; c < 4; ++c)
                {
                    if (std::abs(m_values[r][c] - static_cast<float>(r == c)) > eps) return false;
                }
            }
            return true;
        }

        void transpose()
        {
            m128::transpose(m_columns);
        }

        void transpose3zx()
        {
            m128::transpose3zx(m_columns);
        }

        [[nodiscard]] Matrix4 transposed() const
        {
            Matrix4 m(*this);
            m.transpose();
            return m;
        }

        void invert()
        {
            m128::invertMat4(m_columns);
        }

        [[nodiscard]] Matrix4<float> inverse() const
        {
            Matrix4<float> c(*this);
            c.invert();
            return c;
        }

        void print() const
        {
            for (int r = 0; r < 4; ++r)
            {
                for (int c = 0; c < 4; ++c)
                {
                    std::cout << m_values[c][r] << ' ';
                }
                std::cout << '\n';
            }
        }

    protected:
        union
        {
            __m128 m_columns[4];
            // [col][row]
            float m_values[4][4];
        };

        Matrix4(__m128 c0, __m128 c1, __m128 c2, __m128 c3) noexcept :
            m_columns{ c0, c1, c2, c3 }
        {

        }

        Matrix4(__m128 c0, __m128 c1, __m128 c2) noexcept :
            m_columns{ c0, c1, c2, _mm_set_ps(1.0f, 0.0f, 0.0f, 0.0f) }
        {

        }

        Matrix4(float xs, float ys, float zs) noexcept :
            Matrix4(
                xs, 0.0f, 0.0f, 0.0f,
                0.0f, ys, 0.0f, 0.0f,
                0.0f, 0.0f, zs, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f
            )
        {
        }

        Matrix4(float xs, float ys, float zs, const Vec3<float>& t) noexcept :
            Matrix4(
                xs, 0.0f, 0.0f, t.x,
                0.0f, ys, 0.0f, t.y,
                0.0f, 0.0f, zs, t.z,
                0.0f, 0.0f, 0.0f, 1.0f
            )
        {
        }

        Matrix4() noexcept :
            Matrix4(
                1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f
            )
        {
        }

        explicit Matrix4(const Vec3<float>& t) noexcept :
            Matrix4(
                1.0f, 0.0f, 0.0f, t.x,
                0.0f, 1.0f, 0.0f, t.y,
                0.0f, 0.0f, 1.0f, t.z,
                0.0f, 0.0f, 0.0f, 1.0f
            )
        {
        }

        explicit Matrix4(const OrthonormalBasis3<float>& basis) noexcept:
            Matrix4(
                m128::truncate3(basis.x().xmm),
                m128::truncate3(basis.y().xmm),
                m128::truncate3(basis.z().xmm)
            )
        {
            transpose3zx();
        }

        explicit Matrix4(const Basis3<float>& basis) noexcept :
            Matrix4(
                m128::truncate3(basis.x().xmm),
                m128::truncate3(basis.y().xmm),
                m128::truncate3(basis.z().xmm)
            )
        {
            transpose3zx();
        }

        Matrix4(const OrthonormalBasis3<float>& basis, const Vec3<float>& t) noexcept :
            Matrix4(
                m128::truncate3(basis.x().xmm),
                m128::truncate3(basis.y().xmm),
                m128::truncate3(basis.z().xmm),
                _mm_set_ps(1.0f, t.z, t.y, t.x)
            )
        {
            transpose3zx();
        }

        Matrix4(const Basis3<float>& basis, const Vec3<float>& t) noexcept :
            Matrix4(
                m128::truncate3(basis.x().xmm),
                m128::truncate3(basis.y().xmm),
                m128::truncate3(basis.z().xmm),
                _mm_set_ps(1.0f, t.z, t.y, t.x)
            )
        {
            transpose3zx();
        }

        explicit Matrix4(const Quat4<float>& q) noexcept :
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

        Matrix4(const Quat4<float>& q, const Vec3<float>& t) noexcept :
            Matrix4(q)
        {
            m_values[3][0] = t.x;
            m_values[3][1] = t.y;
            m_values[3][2] = t.z;
        }

        Matrix4(const Quat4<float>& q, const Point3<float>& origin) noexcept :
            Matrix4(q, Vec3<float>(origin))
        {
            // (TR)T^-1
            // TR done in the constructor

            // R | T     1 | -T     R | -RT+T
            // -----  *  ------  =  ---------
            // 0 | 1     0 | 1      0 |   1

            const Vec3<float> t(m128::sub(origin.xmm, m128::mulMat3Vec3(m_columns, origin.xmm)));
            m_values[3][0] = t.x;
            m_values[3][1] = t.y;
            m_values[3][2] = t.z;
        }
    };

    static_assert(sizeof(Matrix4<float>) == sizeof(__m128) * 4);

    using Matrix4f = Matrix4<float>;
}
