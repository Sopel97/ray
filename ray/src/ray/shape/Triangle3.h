#pragma once

#include "Box3.h"

#include <ray/material/TexCoords.h>

#include <ray/math/BarycentricCoords.h>
#include <ray/math/Vec3.h>

namespace ray
{
    // Allows per vertex normals, but requires their directions
    // to be coherent with direction of cross3(m_e01, m_e02).
    // NOTE: this assumption could be removed by doing one more dot3 in raycast
    // Size should be 128 with no precomputed data.
    struct Triangle3
    {
        Triangle3(const Point3f& v0, const Point3f& v1, const Point3f& v2) noexcept :
            m_v0(v0),
            m_e01(v1 - v0),
            m_e02(v2 - v0),
            m_uvs{ {0, 0}, {1, 0}, {1, 1} }
        {
            m_normals[0] = m_normals[1] = m_normals[2] = Normal3f(cross(m_e01, m_e02).normalized());
        }
        
        Triangle3(
            const Point3f& v0, const Point3f& v1, const Point3f& v2,
            const Normal3f& n0, const Normal3f& n1, const Normal3f& n2
        ) noexcept :
            m_v0(v0),
            m_e01(v1 - v0),
            m_e02(v2 - v0),
            m_uvs{ {0, 0}, {1, 0}, {1, 1} }
        {
        }

        Triangle3(
            const Point3f& v0, const Point3f& v1, const Point3f& v2,
            const Normal3f& n0, const Normal3f& n1, const Normal3f& n2,
            const TexCoords& uv0, const TexCoords& uv1, const TexCoords& uv2
            ) noexcept :
            m_v0(v0),
            m_e01(v1 - v0),
            m_e02(v2 - v0),
            m_normals{ n0, n1, n2 },
            m_uvs { uv0, uv1, uv2 }
        {
        }

        // only used when calculating uv, not in raycast
        // don't do precomputation to save space
        [[nodiscard]] BarycentricCoords barycentric(const Point3f& p) const
        {
            const Vec3f v0p = p - m_v0;
            const float m_d00 = dot(m_e01, m_e01);
            const float m_d01 = dot(m_e01, m_e02);
            const float m_d11 = dot(m_e02, m_e02);
            const float d20 = dot(v0p, m_e01);
            const float d21 = dot(v0p, m_e02);
            const float m_invDet = 1.0f / (m_d00 * m_d11 - m_d01 * m_d01);
            const float v = (m_d11 * d20 - m_d01 * d21) * m_invDet;
            const float w = (m_d00 * d21 - m_d01 * d20) * m_invDet;
            const float u = 1.0f - v - w;
            return { u, v, w };
        }

        [[nodiscard]] Point3f center() const
        {
            return m_v0 + (m_e01 + m_e02) * 0.33333333333333f;
        }

        [[nodiscard]] const Point3f& v0() const
        {
            return m_v0;
        }

        [[nodiscard]] Point3f v1() const
        {
            return m_v0 + m_e01;
        }

        [[nodiscard]] Point3f v2() const
        {
            return m_v0 + m_e02;
        }

        [[nodiscard]] const Vec3f& e01() const
        {
            return m_e01;
        }

        [[nodiscard]] const Vec3f& e02() const
        {
            return m_e02;
        }

        [[nodiscard]] const Normal3f& normal(int i) const
        {
            return m_normals[i];
        }

        [[nodiscard]] const TexCoords& uv(int i) const
        {
            return m_uvs[i];
        }

    private:
        Point3f m_v0;
        Vec3f m_e01;
        Vec3f m_e02;
        Normal3f m_normals[3];
        TexCoords m_uvs[3];
    };
}
