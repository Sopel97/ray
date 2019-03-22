#pragma once

#include "Box3.h"

#include <ray/math/Vec3.h>

namespace ray
{
    struct Disc3
    {
        Point3f origin;
        Normal3f normal;
        float distance; // distance to (0, 0, 0)
        float radius;

        Disc3() noexcept = default;

        Disc3(const Point3f& point, const Normal3f& normal, float radius) noexcept :
            origin(point),
            normal(normal),
            distance(dot(normal, point)),
            radius(radius)
        {
        }

        Disc3(const Point3f& p, const Vec3f& v1, const Vec3f& v2, float radius) noexcept :
            origin(p),
            normal(cross(v1, v2).normalized()),
            distance(dot(normal, p)),
            radius(radius)
        {
        }

        [[nodiscard]] const Point3f& center() const
        {
            return origin;
        }
    };
}
