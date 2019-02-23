#pragma once

#include "Box3.h"

#include <ray/math/Vec3.h>

namespace ray
{
    struct Capsule
    {
        Point3f begin;
        Normal3f axis;
        float length;
        float radius;

        Capsule() = default;

        Capsule(const Point3f& begin, const Point3f& end, float radius) :
            begin(begin),
            axis((end - begin).normalized()),
            length((end - begin).length()),
            radius(radius)
        {
        }

        Point3f center() const
        {
            return begin + axis * (0.5f * length);
        }

        Box3 aabb() const
        {
            const Vec3f halfExtent(radius, radius, radius);
            const Point3f end = begin + axis * length;
            Box3 bb(begin - halfExtent, begin + halfExtent);
            bb.extend(Box3(end - halfExtent, end + halfExtent));
            return bb;
        }

        Vec3f extent() const
        {
            return axis * length;
        }
    };
}
