#pragma once

#include "Box3.h"

#include <ray/math/Vec3.h>

namespace ray
{
    struct Capsule
    {
        Point3f begin;
        Point3f end;
        float radius;

        Capsule() = default;

        Capsule(const Point3f& begin, const Point3f& end, float radius) :
            begin(begin),
            end(end),
            radius(radius)
        {
        }

        Point3f center() const
        {
            return begin + (end - begin) * 0.5f;
        }

        Box3 aabb() const
        {
            const Vec3f halfExtent(radius, radius, radius);
            Box3 bb(begin - halfExtent, begin + halfExtent);
            bb.extend(Box3(end - halfExtent, end + halfExtent));
            return bb;
        }

        Vec3f axis() const
        {
            return end - begin;
        }
    };
}
