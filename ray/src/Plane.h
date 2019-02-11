#pragma once

#include "Vec3.h"

namespace ray
{
    struct Plane
    {
        Normal3f normal;
        float distance; // distance to origin

        constexpr Plane(const Normal3f& normal, float distance) :
            normal(normal),
            distance(distance)
        {
        }
    };
}