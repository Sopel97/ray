#pragma once

#include <ray/math/Vec3.h>

namespace ray
{
    struct Plane
    {
        Normal3f normal;
        float distance; // distance to origin

        Plane(const Normal3f& normal, float distance) :
            normal(normal),
            distance(distance)
        {
        }
    };
}
