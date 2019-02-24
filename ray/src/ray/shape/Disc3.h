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

        Disc3() = default;

        Disc3(const Point3f& point, const Normal3f& normal, float radius) :
            origin(point),
            normal(normal),
            distance(dot(normal, Vec3f(point))),
            radius(radius)
        {
        }

        Disc3(const Point3f& p, const Vec3f& v1, const Vec3f& v2, float radius) :
            origin(p),
            normal(cross(v1, v2).normalized()),
            distance(dot(normal, Vec3f(p))),
            radius(radius)
        {
        }

        const Point3f& center() const
        {
            return origin;
        }
    };
}
