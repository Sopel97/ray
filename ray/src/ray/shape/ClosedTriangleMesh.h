#pragma once

#include "Box3.h"
#include "Triangle3.h"

#include <ray/material/Material.h>
#include <ray/material/TexCoords.h>

#include <ray/math/BarycentricCoords.h>
#include <ray/math/Vec3.h>

#include <array>
#include <vector>

namespace ray
{
    struct ClosedTriangleMesh;

    struct ClosedTriangleMeshVertex
    {
        Point3f point;
        Normal3f normal;
        TexCoords uv;
    };

    struct ClosedTriangleMeshFace
    {
        ClosedTriangleMeshFace(const ClosedTriangleMesh& mesh, int i0, int i1, int i2);

        const ClosedTriangleMeshVertex& vertex(int i) const
        {
            return *m_vertices[i];
        }

        // only used when calculating uv, not in raycast
        // don't do precomputation to save space
        BarycentricCoords barycentric(const Point3f& p) const
        {
            const Point3f v0 = vertex(0).point;
            const Vec3f v0p = p - v0;
            const Vec3f m_e01 = vertex(1).point - v0;
            const Vec3f m_e02 = vertex(2).point - v0;
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

        Point3f center() const
        {
            return Point3f::origin() + (Vec3f(vertex(0).point) + Vec3f(vertex(1).point) + Vec3f(vertex(2).point)) * 0.333333333333333f;
        }

        Box3 aabb() const
        {
            const Point3f v0 = vertex(0).point;
            const Point3f v1 = vertex(1).point;
            const Point3f v2 = vertex(2).point;
            const Point3f bmin = min(min(v0, v1), v2);
            const Point3f bmax = max(max(v0, v1), v2);
            return Box3(bmin, bmax);
        }

    private:
        std::array<const ClosedTriangleMeshVertex*, 3> m_vertices;
    };

    // NOTE: currently it requires that no vertices get added after any
    //       face is added (due to iterator invalidation).
    //       If that becomes a problem then change do std::deque
    struct ClosedTriangleMesh
    {
    public:

        int addVertex(const ClosedTriangleMeshVertex& vertex)
        {
            int idx = static_cast<int>(m_vertexPool.size());
            m_vertexPool.emplace_back(vertex);
            return idx;
        }

        int addFace(int a, int b, int c, const Material* material)
        {
            int idx = static_cast<int>(m_faces.size());
            m_faces.emplace_back(*this, a, b, c);
            m_materials.emplace_back(material);
            return idx;
        }

        const ClosedTriangleMeshVertex& vertex(int i) const
        {
            return m_vertexPool[i];
        }

        const ClosedTriangleMeshFace& face(int i) const
        {
            return m_faces[i];
        }

        const Material* material(int i) const
        {
            return m_materials[i];
        }

        // loses volume
        Triangle3 faceAsTriangle(int i) const
        {
            const auto& v0 = m_faces[i].vertex(0);
            const auto& v1 = m_faces[i].vertex(1);
            const auto& v2 = m_faces[i].vertex(2);
            return Triangle3(
                v0.point, v1.point, v2.point,
                v0.normal, v1.normal, v2.normal,
                v0.uv, v1.uv, v2.uv);
        }

        int numFaces() const
        {
            return static_cast<int>(m_faces.size());
        }

    private:
        std::vector<ClosedTriangleMeshVertex> m_vertexPool;
        std::vector<ClosedTriangleMeshFace> m_faces;
        std::vector<const Material*> m_materials;
    };
}
