#pragma once

#include "m128/M128Math.h"
#include "m128/M128MatrixOperations.h"

#include "Basis3.h"
#include "Matrix4.h"
#include "OrthonormalBasis3.h"
#include "Quat4.h"
#include "Vec3.h"

#include <iostream>

namespace ray
{
    template <typename T>
    struct Matrix3;

    template <>
    struct alignas(alignof(__m128)) Matrix3<float>
    {
        // takes rows, transposes
        // will mostly be used in source with literals
        // so it is the most intuitive way
        Matrix3(
            float m00, float m10, float m20,
            float m01, float m11, float m21,
            float m02, float m12, float m22
        ) noexcept :
            m_values{
                m00, m01, m02, 0.0f,
                m10, m11, m12, 0.0f,
                m20, m21, m22, 0.0f
            }
        {
        }

        Matrix3(const Matrix4<float>& m4) noexcept :
            m_columns{m4.m_columns[0], m4.m_columns[1], m4.m_columns[2]}
        {
        }

        [[nodiscard]] friend Matrix3<float> operator*(const Matrix3<float>& lhs, const Matrix3<float>& rhs)
        {
            Matrix3<float> ret{};
            m128::mulMat3Mat3(lhs.m_columns, rhs.m_columns, ret.m_columns);
            return ret;
        }

        [[nodiscard]] friend Normal3<float> operator*(const Matrix3<float>& lhs, const Normal3<float>& rhs);

        [[nodiscard]] friend Vec3<float> operator*(const Matrix3<float>& lhs, const UnitVec3<float>& rhs)
        {
            return Vec3<float>(m128::mulMat3Vec3(lhs.m_columns, rhs.xmm));
        }

        [[nodiscard]] friend Vec3<float> operator*(const Matrix3<float>& lhs, const Vec3<float>& rhs)
        {
            return Vec3<float>(m128::mulMat3Vec3(lhs.m_columns, rhs.xmm));
        }

        [[nodiscard]] friend Point3<float> operator*(const Matrix3<float>& lhs, const Point3<float>& rhs)
        {
            return Point3<float>(m128::mulMat3Vec3(lhs.m_columns, rhs.xmm));
        }

        [[nodiscard]] bool isAlmostIdentity() const
        {
            static constexpr float eps = 1e-5;
            for (int r = 0; r < 3; ++r)
            {
                for (int c = 0; c < 3; ++c)
                {
                    if (std::abs(m_values[r][c] - static_cast<float>(r == c)) > eps) return false;
                }
            }
            return true;
        }

        void transpose()
        {
            m128::transpose3zx(m_columns);
        }

        [[nodiscard]] Matrix3 transposed() const
        {
            Matrix3 m(*this);
            m.transpose();
            return m;
        }

        void invert()
        {
            m128::invertMat3(m_columns);
        }

        [[nodiscard]] Matrix3<float> inverse() const
        {
            Matrix3<float> c(*this);
            c.invert();
            return c;
        }

        void print() const
        {
            for (int r = 0; r < 3; ++r)
            {
                for (int c = 0; c < 3; ++c)
                {
                    std::cout << m_values[c][r] << ' ';
                }
                std::cout << '\n';
            }
        }

    protected:
        union
        {
            __m128 m_columns[3];
            // [col][row]
            float m_values[3][4];
        };

        Matrix3(__m128 c0, __m128 c1, __m128 c2) noexcept :
            m_columns{ c0, c1, c2 }
        {

        }

        Matrix3(float xs, float ys, float zs) noexcept :
            Matrix3(
                xs, 0.0f, 0.0f,
                0.0f, ys, 0.0f,
                0.0f, 0.0f, zs
            )
        {
        }

        Matrix3() noexcept :
            Matrix3(
                1.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 1.0f
            )
        {
        }

        explicit Matrix3(const OrthonormalBasis3<float>& basis) noexcept :
            Matrix3(
                m128::truncate3(basis.x().xmm),
                m128::truncate3(basis.y().xmm),
                m128::truncate3(basis.z().xmm)
            )
        {
            transpose();
        }

        explicit Matrix3(const Basis3<float>& basis) noexcept :
            Matrix3(
                m128::truncate3(basis.x().xmm),
                m128::truncate3(basis.y().xmm),
                m128::truncate3(basis.z().xmm)
            )
        {
            transpose();
        }

        Matrix3(const OrthonormalBasis3<float>& basis, const Vec3<float>& t) noexcept :
            Matrix3(
                m128::truncate3(basis.x().xmm),
                m128::truncate3(basis.y().xmm),
                m128::truncate3(basis.z().xmm)
            )
        {
            transpose();
        }

        explicit Matrix3(const Quat4<float>& q) noexcept :
            Matrix3()
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
    };

    static_assert(sizeof(Matrix3<float>) == sizeof(__m128) * 3);

    using Matrix3f = Matrix3<float>;
}
