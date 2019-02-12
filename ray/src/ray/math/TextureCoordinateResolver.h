#pragma once

#include "MathConstants.h"

#include <ray/material/TexCoords.h>

#include <ray/scene/SceneRaycastHit.h>

#include <ray/shape/Box3.h>
#include <ray/shape/Plane.h>
#include <ray/shape/Sphere.h>

#include <cmath>
#include <iostream>

namespace ray
{
    inline TexCoords resolveTexCoords(const Sphere& sphere, const ResolvableRaycastHit& hit, int shapeInPackNo)
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

    inline TexCoords resolveTexCoords(const Plane& plane, const ResolvableRaycastHit& hit, int shapeInPackNo)
    {
        return { hit.point.x, hit.point.z };
    }

    inline TexCoords resolveTexCoords(const Box3& box, const ResolvableRaycastHit& hit, int shapeInPackNo)
    {
        const Vec3f extent = box.extent();
        const Normal3f normal = hit.normal;
        Point3f base = box.min;
        if (normal.x > 0.5f)
        {
            base.x += extent.x;
        }
        else if (normal.y > 0.5f)
        {
            base.y += extent.y;
        }
        else if (normal.z > 0.5f)
        {
            base.z += extent.z;
        }
        
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
}