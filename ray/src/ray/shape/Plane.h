#pragma once

#include <ray/math/Vec3.h>

namespace ray
{
    struct Plane
    {
        Normal3f normal;
        float distance; // distance to origin

        Plane() noexcept = default;

        Plane(const Normal3f& normal, float distance) noexcept :
            normal(normal),
            distance(distance)
        {
        }

        Plane(const Point3f& p, const Vec3f& v1, const Vec3f& v2) noexcept :
            normal(cross(v1, v2).normalized()),
            distance(dot(normal, Vec3f(p)))
        {
        }
    };
}
