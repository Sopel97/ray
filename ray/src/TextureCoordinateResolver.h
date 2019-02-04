#pragma once

#include "MathConstants.h"
#include "RaycastHit.h"
#include "Sphere.h"
#include "TexCoords.h"

#include <cmath>

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
}
