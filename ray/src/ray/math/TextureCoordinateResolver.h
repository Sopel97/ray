#pragma once

#include "MathConstants.h"

#include <ray/material/TexCoords.h>

#include <ray/scene/SceneRaycastHit.h>

#include <ray/shape/Box3.h>
#include <ray/shape/ClosedTriangleMesh.h>
#include <ray/shape/Capsule.h>
#include <ray/shape/Cylinder.h>
#include <ray/shape/OrientedBox3.h>
#include <ray/shape/Disc3.h>
#include <ray/shape/Triangle3.h>
#include <ray/shape/Plane.h>
#include <ray/shape/Sdf.h>
#include <ray/shape/Sphere.h>
#include <ray/shape/TransformedShape3.h>

#include <cmath>
#include <iostream>

namespace ray
{
    [[nodiscard]] inline TexCoords resolveTexCoords(const Sphere& sphere, const RaycastHit& hit)
    {
        const Normal3f normal = hit.isInside ? -hit.normal : hit.normal;

        // In this particular case, the normal is simular to a point on a unit sphere
        // centred around the origin. We can thus use the normal coordinates to compute
        // the spherical coordinates of Phit.
        // atan2 returns a value in the range [-pi, pi] and we need to remap it to range [0, 1]
        // acosf returns a value in the range [0, pi] and we also need to remap it to the range [0, 1]
        const float u = (1.0f + std::atan2(hit.normal.z, hit.normal.x) / pi) * 0.5f;
        const float v = std::acosf(hit.normal.y) / pi;

        return { u, v };
    }

    // TODO: the following 3 functions
    [[nodiscard]] inline TexCoords resolveTexCoords(const Plane& plane, const RaycastHit& hit)
    {
        return { hit.point.x, hit.point.z };
    }

    [[nodiscard]] inline TexCoords resolveTexCoords(const Disc3& plane, const RaycastHit& hit)
    {
        return { hit.point.x, hit.point.z };
    }

    [[nodiscard]] inline TexCoords resolveTexCoords(const Cylinder& cyl, const RaycastHit& hit)
    {
        return { hit.point.x, hit.point.z };
    }

    [[nodiscard]] inline TexCoords resolveTexCoords(const Capsule& cap, const RaycastHit& hit)
    {
        return { hit.point.x, hit.point.z };
    }

    [[nodiscard]] inline TexCoords resolveTexCoords(const Box3& box, const RaycastHit& hit)
    {
        const Vec3f extent = box.extent();
        const Normal3f normal = hit.normal;
        const Vec3Mask<float> nm = normal > 0.5f;
        const Point3f base = Point3f::blend(box.min, box.max, nm);
        
        const Vec3f diff = (hit.point - base) / extent;
        if (diff.x < diff.y && diff.x < diff.z)
        {
            // x = 0
            return { diff.y, diff.z };
        }
        if (diff.y < diff.x && diff.y < diff.z)
        {
            // y = 0
            return { diff.x, diff.z };
        }
        // z = 0
        return { diff.x, diff.y };
    }

    [[nodiscard]] inline TexCoords resolveTexCoords(const OrientedBox3& obb, const RaycastHit& hit)
    {
        const Vec3f extent = obb.halfSize * 2.0f;
        const Normal3f normal = obb.worldToLocalRot * hit.normal;
        const Point3f p = Point3f::origin() + (hit.point - obb.min());
        const Point3f hitPoint = obb.worldToLocalRot * p;
        const Vec3Mask<float> nm = normal > 0.5f;
        const Point3f base = Point3f::blend(Point3f::origin() - obb.halfSize, Point3f::origin() + obb.halfSize, nm);

        const Vec3f diff = (hitPoint - base) / extent;
        if (diff.x < diff.y && diff.x < diff.z)
        {
            // x = 0
            return { diff.y, diff.z };
        }
        if (diff.y < diff.x && diff.y < diff.z)
        {
            // y = 0
            return { diff.x, diff.z };
        }
        // z = 0
        return { diff.x, diff.y };
    }

    [[nodiscard]] inline TexCoords resolveTexCoords(const Triangle3& tri, const RaycastHit& hit)
    {
        BarycentricCoords bc = tri.barycentric(hit.point);
        return tri.uv(0) * bc.u + tri.uv(1) * bc.v + tri.uv(2) * bc.w;
    }

    [[nodiscard]] inline TexCoords resolveTexCoords(const ClosedTriangleMeshFace& tri, const RaycastHit& hit)
    {
        BarycentricCoords bc = tri.barycentric(hit.point);
        return tri.vertex(0).uv * bc.u + tri.vertex(1).uv * bc.v + tri.vertex(2).uv * bc.w;
    }

    template <typename TransformT, typename ShapeT>
    [[nodiscard]] inline TexCoords resolveTexCoords(const TransformedShape3<TransformT, ShapeT>& sh, const RaycastHit& hit)
    {
        RaycastHit hitLocal = hit;
        hitLocal.point = sh.worldToLocal * hitLocal.point;
        hitLocal.normal = sh.worldToLocal * hitLocal.normal;
        return resolveTexCoords(sh.shape, hitLocal);
    }

    template <typename ClippingShapeT>
    [[nodiscard]] inline TexCoords resolveTexCoords(const ClippedSdf<ClippingShapeT>& sh, const RaycastHit& hit)
    {
        return { 0.0f, 0.0f }; // there's no meaningful way to do it, really
    }
}
