#pragma once

#include <ray/math/Vec3.h>

#include <ray/shape/Box3.h>

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
            distance(Vec3f(origin).length()),
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

        Box3 aabb() const
        {
            Vec3f halfExtent(radius, radius, radius);
            // NOTE: not sure if this is right
            halfExtent *= Vec3f::broadcast(1.0f) - abs(Vec3f(normal));
            return Box3(origin - halfExtent, origin + halfExtent);
        }
    };
}
